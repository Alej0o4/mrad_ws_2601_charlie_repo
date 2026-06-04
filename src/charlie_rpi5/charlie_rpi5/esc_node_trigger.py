#!/usr/bin/env python3
"""
esc_node_trigger.py  --  ROS 2 node for PC <-> STM32G431 ESC communication.

Mixed-input control (Logitech F710 XInput):
  IDLE / START phase  â†’ RB button + left stick direction drive the rev-up
    RB (buttons[5]) held + stick forward  (axes[1] >  deadband) â†’ +u_revup
    RB (buttons[5]) held + stick backward (axes[1] < -deadband) â†’ -u_revup
    RB released â†’ u_target = 0  (firmware stops motor)

  RUN phase           â†’ left stick takes over via /cmd_vel_stamped
    linear.x > 0  forward proportional torque
    linear.x < 0  reverse proportional torque
    neutral       â†’ u_target = 0  (firmware handles decel)

  Transition STARTâ†’RUN is seamless: slew rate bridges the torque level.

Control features:
  - u_max clamp        : caps output (default 0.40)
  - u_min_start floor  : enforces minimum during START to reach BEMF floor (~2400 RPM)
  - u_min_run floor    : enforces minimum during RUN to prevent closed-loop dropout
  - Slew rate limiter  : limits du/dt per cycle (default 0.08 at 20 Hz)
  - cmd_hz = 20 Hz

Protocol
--------
Command frame  (PC -> STM32)  5 bytes:
  [0xAA] [cmd_lo] [cmd_hi] [0x00] [XOR_chk]
   SOF    int16_t u  (-32768=-1.0 / +32767=+1.0)   XOR(bytes 1..3)

Telemetry frame (STM32 -> PC) 15 bytes:
  [0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi]
        [iq_lo][iq_hi][id_lo][id_hi][mcsdk_st][XOR of bytes 1..13]
   spd      int16   RPM (average mechanical speed)
   esc_st   uint8   ESC state (see below)
   faults   uint8   lower byte of MC_GetCurrentFaultsMotor1()
   u        int16   raw command (-32768..+32767)
   v        uint16  DC bus voltage in whole Volts
   iq       int16   q-axis current in milliAmps
   id       int16   d-axis current in milliAmps
   mcsdk_st uint8   MCI_State_t: 0=IDLE 4=START 6=RUN 10=FAULT_NOW 11=FAULT_OVER

ESC state byte:
  0=BOOT  1=WAIT_NEUTRAL  2=READY  3=FORWARD  4=BRAKE  5=REVERSE  6=FAULT
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int16, String

import serial

# -- Protocol constants --------------------------------------------------------
CMD_SOF       = 0xAA
TLM_SOF       = 0xBB
CMD_FRAME_LEN = 5
TLM_FRAME_LEN = 15

# F710 XInput indices
JOY_BTN_RB    = 5  # Right bumper â†’ activates rev-up
JOY_AXIS_LS_Y = 1  # Left stick Y â†’ direction (+1=forward, -1=backward)

STATE_NAMES = {
    0: 'BOOT',
    1: 'WAIT_NEUTRAL',
    2: 'READY',
    3: 'FORWARD',
    4: 'BRAKE',
    5: 'REVERSE',
    6: 'FAULT',
}

FAULT_BITS = {
    0x02: 'OVER_VOLT',
    0x04: 'UNDER_VOLT',
    0x08: 'OVER_TEMP',
    0x10: 'START_UP',
    0x20: 'SPEED_FDBK',
    0x40: 'OVER_CURR',
    0x80: 'SW_ERROR',
}

MCSDK_STATE_NAMES = {
    0:  'IDLE',
    4:  'START',
    6:  'RUN',
    10: 'FAULT_NOW',
    11: 'FAULT_OVER',
}

# -- Frame builders / decoders -------------------------------------------------

def build_command(u: float) -> bytes:
    """Encode normalised command u in [-1.0, +1.0] into a 5-byte frame."""
    u   = max(-1.0, min(1.0, u))
    raw = int(u * 32767)
    lo  = raw & 0xFF
    hi  = (raw >> 8) & 0xFF
    chk = lo ^ hi ^ 0x00
    return bytes([CMD_SOF, lo, hi, 0x00, chk])


def decode_state(b: int) -> str:
    return STATE_NAMES.get(b, f'STATE_{b}')


def decode_faults(b: int) -> str:
    if b == 0:
        return 'none'
    return ' | '.join(name for bit, name in FAULT_BITS.items() if b & bit) or f'0x{b:02X}'


def parse_telemetry(frame: bytes):
    """Return (speed_rpm, state_str, fault_str, u_float, vbus_v, iq_ma, id_ma, mcsdk_state_str)
    or None on bad checksum."""
    if len(frame) != TLM_FRAME_LEN or frame[0] != TLM_SOF:
        return None
    chk = 0
    for b in frame[1:14]:
        chk ^= b
    if chk != frame[14]:
        return None
    speed = struct.unpack_from('<h', frame, 1)[0]
    cmd_r = struct.unpack_from('<h', frame, 5)[0]
    vbus  = struct.unpack_from('<H', frame, 7)[0]
    iq_ma = struct.unpack_from('<h', frame, 9)[0]
    id_ma = struct.unpack_from('<h', frame, 11)[0]
    u_val = cmd_r / 32767.0
    mcsdk_state = MCSDK_STATE_NAMES.get(frame[13], f'MCSDK_{frame[13]}')
    return speed, decode_state(frame[3]), decode_faults(frame[4]), u_val, vbus, iq_ma, id_ma, mcsdk_state


# -- ROS 2 Node ----------------------------------------------------------------

class EscNodeTrigger(Node):

    def __init__(self):
        super().__init__('esc_node')

        # Parameters
        self.declare_parameter('port',      '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF555567894967082109-if02')
        self.declare_parameter('baudrate',  1843200)
        self.declare_parameter('cmd_hz',      20)    # sender rate (Hz)
        self.declare_parameter('timeout',      0.5)  # s â€” stop if active input goes silent
        self.declare_parameter('deadband',     0.05) # min input value to act
        self.declare_parameter('max_speed',    1.0)  # m/s that maps to u=1.0 (stick scaling)
        self.declare_parameter('u_max',        0.40) # hard output cap
        self.declare_parameter('u_revup',      0.30) # fixed command sent while bumper held
        self.declare_parameter('u_min_start',  0.25) # floor during START â†’ reach BEMF floor
        self.declare_parameter('u_min_run',    0.10) # floor during RUN â†’ prevent dropout
        self.declare_parameter('slew_rate',    0.08) # max |du| per sender cycle

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self._cmd_hz      = self.get_parameter('cmd_hz').value
        self._timeout     = self.get_parameter('timeout').value
        self._deadband    = self.get_parameter('deadband').value
        self._max_speed   = self.get_parameter('max_speed').value
        self._u_max       = self.get_parameter('u_max').value
        self._u_revup     = self.get_parameter('u_revup').value
        self._u_min_start = self.get_parameter('u_min_start').value
        self._u_min_run   = self.get_parameter('u_min_run').value
        self._slew_rate   = self.get_parameter('slew_rate').value

        # Shared state
        self._u_target      = 0.0    # desired command from active input
        self._u_current     = 0.0    # slew-limited output sent to ESC
        self._mcsdk_raw     = 'IDLE' # latest mcsdk_state from telemetry
        self._u_lock        = threading.Lock()
        self._last_joy_time = time.time()
        self._last_cmd_time = time.time()

        # Open serial port
        self.get_logger().info(f'Opening {port} at {baudrate} baud ...')
        try:
            self._ser = serial.Serial(port, baudrate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise SystemExit(1)
        self.get_logger().info('Serial port open.')
        self.get_logger().info(
            'IDLE/START: hold RB + stick fwd/back to rev-up  |  RUN: left stick via /cmd_vel_stamped')

        # Publishers
        self._pub_speed       = self.create_publisher(Int16,   'esc/speed_rpm',   10)
        self._pub_state       = self.create_publisher(String,  'esc/state',       10)
        self._pub_faults      = self.create_publisher(String,  'esc/faults',      10)
        self._pub_cmd         = self.create_publisher(Float32, 'esc/command',     10)
        self._pub_vbus        = self.create_publisher(Float32, 'esc/vbus_v',      10)
        self._pub_iq_ma       = self.create_publisher(Int16,   'esc/iq_ma',       10)
        self._pub_id_ma       = self.create_publisher(Int16,   'esc/id_ma',       10)
        self._pub_mcsdk_state = self.create_publisher(String,  'esc/mcsdk_state', 10)

        # Subscribers â€” both active simultaneously; mcsdk state gates which one drives
        self._sub_joy = self.create_subscription(Joy, 'joy', self._joy_cb, 10)
        self._sub_cmd = self.create_subscription(
            TwistStamped, 'cmd_vel_stamped', self._cmd_vel_cb, 10)

        # Background threads
        self._stop   = threading.Event()
        self._t_send = threading.Thread(target=self._sender_thread, daemon=True)
        self._t_recv = threading.Thread(target=self._reader_thread, daemon=True)
        self._t_send.start()
        self._t_recv.start()

    # -- Subscriber callbacks --------------------------------------------------

    def _joy_cb(self, msg: Joy):
        """RB + left stick direction active during IDLE/START phase only."""
        if len(msg.buttons) <= JOY_BTN_RB or len(msg.axes) <= JOY_AXIS_LS_Y:
            return

        self._last_joy_time = time.time()

        with self._u_lock:
            if self._mcsdk_raw == 'RUN':
                return  # stick has control in RUN â€” ignore buttons

        rb    = msg.buttons[JOY_BTN_RB]        # 1=held, 0=released
        ls_y  = msg.axes[JOY_AXIS_LS_Y]        # +1=forward, -1=backward

        if rb and ls_y > self._deadband:
            target = self._u_revup              # forward rev-up
        elif rb and ls_y < -self._deadband:
            target = -self._u_revup             # reverse rev-up
        else:
            target = 0.0                        # RB released or stick neutral

        with self._u_lock:
            self._u_target = target

    def _cmd_vel_cb(self, msg: TwistStamped):
        """Left stick active during RUN phase only."""
        self._last_cmd_time = time.time()

        with self._u_lock:
            if self._mcsdk_raw != 'RUN':
                return  # triggers have control outside RUN â€” ignore stick

        joy_norm = max(-1.0, min(1.0, msg.twist.linear.x / self._max_speed))

        if abs(joy_norm) < self._deadband:
            target = 0.0
        else:
            direction = 1.0 if joy_norm > 0 else -1.0
            scale = (abs(joy_norm) - self._deadband) / (1.0 - self._deadband)
            target = direction * min(scale * self._u_max, self._u_max)

        with self._u_lock:
            self._u_target = target

    # -- Background threads ----------------------------------------------------

    def _sender_thread(self):
        """Applies slew rate, mcsdk-aware floors, timeout, sends at cmd_hz."""
        interval = 1.0 / self._cmd_hz
        while not self._stop.is_set():
            now = time.time()

            with self._u_lock:
                mcsdk = self._mcsdk_raw

                # Timeout: use the relevant input source for the current phase
                if mcsdk == 'RUN':
                    timed_out = (now - self._last_cmd_time) > self._timeout
                else:
                    timed_out = (now - self._last_joy_time) > self._timeout

                if timed_out:
                    self._u_target = 0.0

                target  = self._u_target
                current = self._u_current

                # Slew rate limit
                delta = target - current
                delta = max(-self._slew_rate, min(self._slew_rate, delta))
                self._u_current = current + delta

                u_out = self._u_current

            # mcsdk-aware torque floors (applied after slew, outside lock)
            if u_out != 0.0:
                sign = 1.0 if u_out > 0 else -1.0
                if mcsdk == 'START' and abs(u_out) < self._u_min_start:
                    u_out = sign * self._u_min_start
                elif mcsdk == 'RUN' and abs(u_out) < self._u_min_run:
                    u_out = sign * self._u_min_run

            try:
                self._ser.write(build_command(u_out))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                break
            time.sleep(interval)

    def _reader_thread(self):
        """Reads telemetry frames and publishes topics."""
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= TLM_FRAME_LEN:
                if buf[0] != TLM_SOF:
                    buf.pop(0)
                    continue
                frame = bytes(buf[:TLM_FRAME_LEN])
                result = parse_telemetry(frame)
                if result is None:
                    buf.pop(0)
                    continue
                speed, state, faults, u_val, vbus, iq_ma, id_ma, mcsdk_state = result
                with self._u_lock:
                    self._mcsdk_raw = mcsdk_state
                self.get_logger().debug(
                    f'spd={speed} RPM  state={state}  mcsdk={mcsdk_state}  '
                    f'u={u_val:+.3f}  vbus={vbus} V  '
                    f'Iq={iq_ma} mA  Id={id_ma} mA  faults={faults}'
                )
                self._pub_speed.publish(Int16(data=speed))
                self._pub_state.publish(String(data=state))
                self._pub_faults.publish(String(data=faults))
                self._pub_cmd.publish(Float32(data=float(u_val)))
                self._pub_vbus.publish(Float32(data=float(vbus)))
                self._pub_iq_ma.publish(Int16(data=iq_ma))
                self._pub_id_ma.publish(Int16(data=id_ma))
                self._pub_mcsdk_state.publish(String(data=mcsdk_state))
                buf = buf[TLM_FRAME_LEN:]

    # -- Shutdown --------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Sending neutral and closing serial port ...')
        with self._u_lock:
            self._u_target  = 0.0
            self._u_current = 0.0
        try:
            self._ser.write(build_command(0.0))
        except serial.SerialException:
            pass
        time.sleep(0.2)
        self._stop.set()
        self._ser.close()
        super().destroy_node()


# -- Entry point ---------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = EscNodeTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()