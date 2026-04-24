#!/usr/bin/env python3
"""
esc_node_custom_foc.py  --  ROS 2 ESC node for custom FOC firmware.

cmd_vel_stamped control (geometry_msgs/TwistStamped):
  Two-phase control based on CFOC state from telemetry:

  STARTUP (IDLE → ALIGNMENT → OPEN_LOOP → CROSSFADE):
    Non-zero linear.x → send fixed u_startup (direction only).
    The startup command is constant so the open-loop ramp is clean.
    linear.x = 0 → neutral → firmware stops motor.

  CLOSED_LOOP:
    u = clamp(linear_x / max_linear_mps, -u_max, u_max)
    Slew rate limiter smooths the transition from startup to proportional.

  Timeout: if no cmd_vel_stamped received for cmd_vel_timeout seconds,
    u is forced to neutral.

Telemetry (STM32 → ROS2) published at 10 Hz.
Bag recording: ros2 bag record -a -o bag_file/test_name

Runtime config (sent once at startup via 0xCC frames):
  max_speed_rpm:  u=1.0 maps to this RPM (default 3000, range 1000-10000)
  iq_limit_a:     speed PI current clamp in A (default 12.0, range 1.0-20.0)
  xf_duration_ms: crossfade blend duration in ms (default 500, range 100-1000)
  xf_dwell_ms:    crossfade dwell time in ms (default 200, range 50-500)
  ol_target_rpm:  OL target / crossfade speed in RPM (default 1400, range 1000-2000)
  spd_kp:         speed PI Kp in A/RPM (default 0.01, range 0.005-0.050)
  spd_ki:         speed PI Ki in A/RPM (default 0.001, range 0.0005-0.0050)
  spd_lpf_alpha:  speed PI LPF alpha (default 0.0004, range 0.0002-0.0020)

Protocol — same 5-byte command / 15-byte telemetry as esc_comm.h.
CFOC states: 0=IDLE 1=ALIGNMENT 2=OPEN_LOOP 3=CROSSFADE 4=CLOSED_LOOP 5=FAULT

--------------------------------------------------------------------------------
HOSIM tuning notes (2026-04-15) — AMORIL → HOSIM port
--------------------------------------------------------------------------------
AMORIL #1 is the baseline car (lighter drivetrain, validated 04-09, bag
rosbag2_2026_04_09-11_42_34). HOSIM #1 has a stiffer / heavier drivetrain and
needs stronger OL current + higher crossfade RPM to keep the observer locked,
especially in REVERSE.

Parameter deltas from AMORIL baseline to current HOSIM working set:
  ol_iq_a         8.0  → 12.0   (break stiction / hold speed against load)
  ol_ramp_ms      4000 → 4000   (same; longer ramp hurts UX, no observer win)
  ol_target_rpm   1400 → 1800   (more BEMF headroom; REV observer was losing
                                 lock at 1400 under ground load)
  align_ms        500  → 1000   (heavier rotor — needs longer alignment)
  align_id_a      5.0  → 10.0   (same reason — break stiction)
  xf_duration_ms  200  → 300    (longer blend → smoother CL hand-off)
  xf_dwell_ms     100  → 300    (more observer-lock time before CL handover)
  spd_kp          0.02 → 0.015  (softer PI to tame saturated post-CL response)
  spd_ki          0.002→ 0.001
  spd_lpf_alpha   0.001→ 0.0006 (smoother filter under noisier sensor)
  iq_limit_a      —    → 20.0   (remove 15 A saturation ceiling; load rarely
                                 demands more than 16 A, but leaves headroom)

Joystick ergonomics fix (2026-04-15):
  Added 'cl_mode' parameter — default 'full'. In 'full' mode, ANY non-deadband
  stick deflection triggers the staged startup (node forces u = u_startup until
  firmware reports CLOSED_LOOP, then gently ramps to ±u_max). User no longer
  has to precisely hold the stick at ±0.3 for the 4 s OL ramp and then push to
  full throttle — it is now "push & hold → release" only.
  Set cl_mode='proportional' to restore the old proportional-cmd_vel behavior
  (for autonomy / nav2 control).

Bumpless CL entry (2026-04-15, hardware-motivated):
  On ground, every CL entry was collapsing ~1000+ RPM (e.g. OL 1937 → CL
  817 RPM) because the speed PI setpoint at CL entry = u_startup × max_speed
  = 0.3 × 3000 = 900 RPM, but the motor was physically at ~1800 RPM. PI saw
  "too fast" → braked → driver felt the car stop.
  Fix: on CL entry, snap u_current to ol_target_rpm / max_speed_rpm so the
  setpoint matches the handover speed. No negative-Iq brake hit. Then the
  cl_post_ramp_ms ramp takes over and smoothly pushes u to u_max.

Progressive-throttle mapping (2026-04-22, cl_mode='full'):
  In CL, stick magnitude is now mapped proportionally into the
  [u_bumpless, u_max] saturation band: u_stick = u_bumpless + |vx|/max_linear *
  (u_max - u_bumpless). The bumpless entry ramp still runs for cl_post_ramp_ms,
  but its endpoint tracks the live stick target instead of a fixed u_max=1.0.
  Previously any non-deadband stick ramped to 1.0 regardless, giving a
  bang-bang feel; this restores progressive throttle in full mode without
  losing bumpless CL entry.

Post-CL gentle ramp (2026-04-15, hardware-motivated):
  On bench, holding stick at ±1.0 at the instant of CL entry caused grinding
  and stall — the speed PI saw a 1200 RPM step error (OL target 1800 → CL
  setpoint u_max*3000 = 3000) and saturated, producing torque chatter.
  Mid-stick operation let the motor breathe through CL. On ground this wasn't
  reproducible manually.
  Fix: added 'cl_post_ramp_ms' (default 1500 ms). In cl_mode='full', u is
  ramped smoothly from u_startup to u_max over this window after CL entry,
  giving the speed PI time to track without saturation.

Recommended HOSIM CLI for ground tuning:
  ros2 run golf_rc_car_motion esc_node_custom_foc --ros-args \
    -p ol_iq_a:=12.0 -p ol_ramp_ms:=4000 -p ol_target_rpm:=1800 \
    -p xf_duration_ms:=300 -p xf_dwell_ms:=300 \
    -p align_ms:=1000 -p align_id_a:=10.0 \
    -p spd_kp:=0.015 -p spd_ki:=0.001 -p spd_lpf_alpha:=0.0006 \
    -p iq_limit_a:=20.0 -p cl_mode:=full

Known HOSIM limitations (as of 2026-04-15):
  * REV CL-entry was ~0% on HOSIM before these changes. First clean REV
    transition observed in bag rosbag2_2026_04_15-09_35_19 (18 RPM drop at
    ol_target_rpm=1800) — needs more samples to confirm repeatability.
  * FWD CL-entry quality variable: 50 RPM drop (clean) to 1200 RPM (rough),
    depends on whether OL reached 1800 before crossfade.
  * Magnetometer unusable near motor (EMI ~+24 % |B|, heading std 59-104°).
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Int16, String

import serial

# -- Protocol constants --------------------------------------------------------
CMD_SOF       = 0xAA
TLM_SOF       = 0xBB
CFG_SOF       = 0xCC
CMD_FRAME_LEN = 5
TLM_FRAME_LEN = 21

# Config param IDs (0xCC frame)
CFG_PARAM_MAX_IQ    = 0x01
CFG_PARAM_REVUP     = 0x02
CFG_PARAM_BOOST_IQ  = 0x03
CFG_PARAM_MAX_SPD   = 0x04
CFG_PARAM_IQ_LIMIT  = 0x05
CFG_PARAM_OL_IQ     = 0x06
CFG_PARAM_OL_RAMP   = 0x07
CFG_PARAM_ALIGN_MS  = 0x08
CFG_PARAM_ALIGN_ID  = 0x09
CFG_PARAM_XF_DUR    = 0x0A
CFG_PARAM_XF_DWELL  = 0x0B
CFG_PARAM_OL_RPM    = 0x0C
CFG_PARAM_SPD_KP    = 0x0D
CFG_PARAM_SPD_KI    = 0x0E
CFG_PARAM_SPD_LPF   = 0x0F
# Step 8 adaptive-R EKF params (firmware ignores when observer_mode=0)
CFG_PARAM_OBS_MODE  = 0x14
CFG_PARAM_EKF_WTHR  = 0x15
CFG_PARAM_EKF_R0    = 0x16
CFG_PARAM_EKF_QE    = 0x17
CFG_PARAM_EKF_VFPL  = 0x18

STATE_NAMES = {
    0: 'BOOT', 1: 'WAIT_NEUTRAL', 2: 'READY',
    3: 'FORWARD', 4: 'BRAKE', 5: 'REVERSE', 6: 'FAULT',
}

CFOC_STATE_NAMES = {
    0: 'IDLE', 1: 'ALIGNMENT', 2: 'OPEN_LOOP',
    3: 'CROSSFADE', 4: 'CLOSED_LOOP', 5: 'FAULT',
}

FAULT_BITS = {
    0x02: 'OVER_VOLT', 0x04: 'UNDER_VOLT', 0x08: 'OVER_TEMP',
    0x10: 'START_UP',  0x20: 'SPEED_FDBK', 0x40: 'OVER_CURR',
    0x80: 'SW_ERROR',
}

# -- Frame builders / decoders ------------------------------------------------

def build_config(param_id: int, value: int) -> bytes:
    """Build a 5-byte 0xCC config frame: [SOF][param_id][val_lo][val_hi][XOR]."""
    val = value & 0xFFFF
    lo = val & 0xFF
    hi = (val >> 8) & 0xFF
    chk = param_id ^ lo ^ hi
    return bytes([CFG_SOF, param_id, lo, hi, chk])


def build_command(u: float) -> bytes:
    u = max(-1.0, min(1.0, u))
    raw = int(u * 32767)
    lo = raw & 0xFF
    hi = (raw >> 8) & 0xFF
    chk = lo ^ hi ^ 0x00
    return bytes([CMD_SOF, lo, hi, 0x00, chk])


def parse_telemetry(frame: bytes):
    if len(frame) != TLM_FRAME_LEN or frame[0] != TLM_SOF:
        return None
    chk = 0
    for b in frame[1:20]:
        chk ^= b
    if chk != frame[20]:
        return None
    speed   = struct.unpack_from('<h', frame, 1)[0]
    cmd_r   = struct.unpack_from('<h', frame, 5)[0]
    vbus    = struct.unpack_from('<H', frame, 7)[0] / 10.0   # tenths of V → V
    iq_ma   = struct.unpack_from('<h', frame, 9)[0]
    id_ma   = struct.unpack_from('<h', frame, 11)[0]
    esc_st  = STATE_NAMES.get(frame[3], f'STATE_{frame[3]}')
    raw_fb  = frame[4]
    cfoc_fault = raw_fb & 0x01
    swap_def   = (raw_fb >> 1) & 0x01
    cl_hyst_ms = (raw_fb >> 2) & 0x3F
    faults  = 'none' if cfoc_fault == 0 else 'CFOC_FAULT'
    cfoc_st = CFOC_STATE_NAMES.get(frame[13], f'CFOC_{frame[13]}')
    innov_a = struct.unpack_from('<H', frame, 14)[0] / 1000.0   # A × 1000 → A
    kappa   = struct.unpack_from('<H', frame, 16)[0] / 10000.0  # × 10000 → dimensionless
    residual = struct.unpack_from('<H', frame, 18)[0] / 1000.0  # V × 1000 → V
    u_val   = cmd_r / 32767.0
    return (speed, esc_st, faults, u_val, vbus, iq_ma, id_ma,
            cfoc_st, innov_a, kappa, residual, swap_def, cl_hyst_ms)


# -- ROS 2 Node ---------------------------------------------------------------

class EscNodeCustomFoc(Node):

    def __init__(self):
        super().__init__('esc_node')

        # Parameters
        self.declare_parameter('port',
            '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF494970535067242339-if02')
        self.declare_parameter('baudrate',  1843200)
        self.declare_parameter('cmd_hz',    20)
        self.declare_parameter('deadband',        0.05)
        self.declare_parameter('u_max',           1.0)
        self.declare_parameter('u_startup',       0.30)   # fixed command during startup
        self.declare_parameter('slew_rate',       0.05)   # max |du| per sender cycle
        self.declare_parameter('max_linear_mps',  1.0)    # linear.x that maps to u=1.0
        self.declare_parameter('cmd_vel_timeout', 0.5)    # seconds before forced neutral
        # Vehicle profile — selects per-car OL current default.
        # Explicit -p ol_iq_a:=X always overrides the profile.
        self.declare_parameter('vehicle', '')           # '' | 'amoril' | 'hosim'
        vehicle = str(self.get_parameter('vehicle').value).lower().strip()
        VEHICLE_OL_IQ_A = {'amoril': 8.0, 'hosim': 15.0}
        ol_iq_default = VEHICLE_OL_IQ_A.get(vehicle, 8.0)

        self.declare_parameter('max_speed_rpm', 3000)   # u=1.0 maps to this RPM
        self.declare_parameter('iq_limit_a',    12.0)   # speed PI Iq clamp [A]
        self.declare_parameter('ol_iq_a',       ol_iq_default)  # open-loop Iq target [A]
        self.declare_parameter('ol_ramp_ms',    4000)   # OL speed ramp duration [ms]
        self.declare_parameter('align_ms',      500)    # alignment duration [ms]
        self.declare_parameter('align_id_a',    5.0)    # alignment d-axis current [A]
        self.declare_parameter('xf_duration_ms', 500)   # crossfade blend duration [ms]
        self.declare_parameter('xf_dwell_ms',    200)   # crossfade dwell time [ms]
        self.declare_parameter('ol_target_rpm', 1400)   # OL target / crossfade speed [RPM]
        self.declare_parameter('spd_kp',        0.01)   # speed PI Kp [A/RPM]
        self.declare_parameter('spd_ki',        0.001)  # speed PI Ki [A/RPM] (discretized)
        self.declare_parameter('spd_lpf_alpha', 0.0004) # speed PI LPF alpha
        self.declare_parameter('cl_mode',       'full') # 'full' | 'proportional'
        self.declare_parameter('cl_post_ramp_ms', 1500) # ms to ramp u_startup → u_max after CL entry (cl_mode=full)
        # Step 8 adaptive-R EKF (only sent when observer_mode=1)
        self.declare_parameter('observer_mode',    0)      # 0=discrete (legacy), 1=adaptive-R
        self.declare_parameter('ekf_omega_thresh', 200)    # elec rad/s, range 50–400
        self.declare_parameter('ekf_r0',           3.33e-3) # A², range 1e-4 .. 1.0
        self.declare_parameter('ekf_qe',           1.0e-2)  # range 1e-4 .. 1.0
        self.declare_parameter('ekf_vf_prior_lock', 1)     # 0/1

        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self._cmd_hz    = self.get_parameter('cmd_hz').value
        self._deadband  = self.get_parameter('deadband').value
        self._u_max     = self.get_parameter('u_max').value
        self._u_startup = self.get_parameter('u_startup').value
        self._slew_rate       = self.get_parameter('slew_rate').value
        self._max_linear_mps  = self.get_parameter('max_linear_mps').value
        self._cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self._max_speed_rpm = self.get_parameter('max_speed_rpm').value
        self._iq_limit_a    = self.get_parameter('iq_limit_a').value
        self._ol_iq_a       = self.get_parameter('ol_iq_a').value
        self._ol_ramp_ms    = self.get_parameter('ol_ramp_ms').value
        self._align_ms      = self.get_parameter('align_ms').value
        self._align_id_a    = self.get_parameter('align_id_a').value
        self._xf_duration_ms = self.get_parameter('xf_duration_ms').value
        self._xf_dwell_ms    = self.get_parameter('xf_dwell_ms').value
        self._ol_target_rpm  = self.get_parameter('ol_target_rpm').value
        self._spd_kp         = self.get_parameter('spd_kp').value
        self._spd_ki         = self.get_parameter('spd_ki').value
        self._spd_lpf_alpha  = self.get_parameter('spd_lpf_alpha').value
        self._cl_mode        = self.get_parameter('cl_mode').value
        self._cl_post_ramp_ms = self.get_parameter('cl_post_ramp_ms').value
        self._observer_mode     = int(self.get_parameter('observer_mode').value)
        self._ekf_omega_thresh  = int(self.get_parameter('ekf_omega_thresh').value)
        self._ekf_r0            = float(self.get_parameter('ekf_r0').value)
        self._ekf_qe            = float(self.get_parameter('ekf_qe').value)
        self._ekf_vf_prior_lock = int(self.get_parameter('ekf_vf_prior_lock').value)

        # State
        self._u_target          = 0.0
        self._u_current         = 0.0
        self._cmd_vel_dir       = 0      # +1 fwd, -1 rev, 0 neutral (from cmd_vel)
        self._last_cmd_vel_time = 0.0    # epoch seconds of last received cmd_vel
        self._cfoc_st           = 'IDLE' # latest CFOC state from telemetry
        self._cl_entry_time     = 0.0    # epoch seconds when CFOC entered CLOSED_LOOP (cl_mode=full ramp)
        self._lock              = threading.Lock()

        # Serial
        self.get_logger().info(f'Opening {port} at {baudrate} baud ...')
        try:
            self._ser = serial.Serial(port, baudrate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port: {e}')
            raise SystemExit(1)
        # Send config frames (0xCC) before any motor command
        self._send_config()
        self.get_logger().info(
            f'Port open. Startup: cmd_vel dir → fixed u={self._u_startup:.2f}. '
            f'Closed-loop: proportional (max_linear={self._max_linear_mps:.2f} m/s). '
            f'Timeout: {self._cmd_vel_timeout:.1f} s.')

        # Publishers
        self._pub_speed      = self.create_publisher(Int16,   'esc/speed_rpm',   10)
        self._pub_state      = self.create_publisher(String,  'esc/state',       10)
        self._pub_faults     = self.create_publisher(String,  'esc/faults',      10)
        self._pub_cmd        = self.create_publisher(Float32, 'esc/command',     10)
        self._pub_vbus       = self.create_publisher(Float32, 'esc/vbus_v',      10)
        self._pub_iq_ma      = self.create_publisher(Int16,   'esc/iq_ma',       10)
        self._pub_id_ma      = self.create_publisher(Int16,   'esc/id_ma',       10)
        self._pub_cfoc_state = self.create_publisher(String,  'esc/cfoc_state',  10)
        self._pub_innov      = self.create_publisher(Float32, 'esc/innov_a',     10)
        self._pub_kappa      = self.create_publisher(Float32, 'esc/lock_kappa',  10)
        self._pub_residual   = self.create_publisher(Float32, 'esc/lock_residual', 10)
        self._pub_swap_def   = self.create_publisher(Int16,   'esc/swap_deferred', 10)
        self._pub_cl_hyst    = self.create_publisher(Int16,   'esc/cl_hyst_ms',   10)

        # cmd_vel_stamped subscriber
        self._sub_cmd_vel = self.create_subscription(
            TwistStamped, 'cmd_vel_stamped', self._cmd_vel_cb, 10)

        # Background threads
        self._stop   = threading.Event()
        self._t_send = threading.Thread(target=self._sender_thread, daemon=True)
        self._t_recv = threading.Thread(target=self._reader_thread, daemon=True)
        self._t_send.start()
        self._t_recv.start()

    # -- Config sender ---------------------------------------------------------

    def _send_config(self):
        """Send 0xCC config frames to firmware."""
        cfg_frames = []

        # Speed / torque config
        spd = int(self._max_speed_rpm)
        if 1000 <= spd <= 10000:
            cfg_frames.append(('max_spd', build_config(CFG_PARAM_MAX_SPD, spd)))
        iq = int(round(self._iq_limit_a * 10))
        if 10 <= iq <= 200:
            cfg_frames.append(('iq_lim', build_config(CFG_PARAM_IQ_LIMIT, iq)))

        # Startup config
        ol_iq = int(round(self._ol_iq_a * 10))
        if 20 <= ol_iq <= 150:
            cfg_frames.append(('ol_iq', build_config(CFG_PARAM_OL_IQ, ol_iq)))
        ol_ramp = int(self._ol_ramp_ms)
        if 1000 <= ol_ramp <= 8000:
            cfg_frames.append(('ol_ramp', build_config(CFG_PARAM_OL_RAMP, ol_ramp)))
        al_ms = int(self._align_ms)
        if 100 <= al_ms <= 2000:
            cfg_frames.append(('align_ms', build_config(CFG_PARAM_ALIGN_MS, al_ms)))
        al_id = int(round(self._align_id_a * 10))
        if 10 <= al_id <= 150:
            cfg_frames.append(('align_id', build_config(CFG_PARAM_ALIGN_ID, al_id)))

        # Crossfade / transition tuning params
        xf_dur = int(self._xf_duration_ms)
        if 100 <= xf_dur <= 1000:
            cfg_frames.append(('xf_dur', build_config(CFG_PARAM_XF_DUR, xf_dur)))
        xf_dw = int(self._xf_dwell_ms)
        if 50 <= xf_dw <= 500:
            cfg_frames.append(('xf_dwell', build_config(CFG_PARAM_XF_DWELL, xf_dw)))
        ol_rpm = int(self._ol_target_rpm)
        if 1000 <= ol_rpm <= 2000:
            cfg_frames.append(('ol_rpm', build_config(CFG_PARAM_OL_RPM, ol_rpm)))
        # spd_kp: encoded as int16 × 0.001 (e.g. 0.01 → 10)
        kp_enc = int(round(self._spd_kp / 0.001))
        if 5 <= kp_enc <= 50:
            cfg_frames.append(('spd_kp', build_config(CFG_PARAM_SPD_KP, kp_enc)))
        # spd_ki: encoded as int16 × 0.0001 (e.g. 0.001 → 10)
        ki_enc = int(round(self._spd_ki / 0.0001))
        if 5 <= ki_enc <= 50:
            cfg_frames.append(('spd_ki', build_config(CFG_PARAM_SPD_KI, ki_enc)))
        # spd_lpf_alpha: encoded as int16 × 0.0001 (e.g. 0.0004 → 4)
        lpf_enc = int(round(self._spd_lpf_alpha / 0.0001))
        if 2 <= lpf_enc <= 20:
            cfg_frames.append(('spd_lpf', build_config(CFG_PARAM_SPD_LPF, lpf_enc)))

        # Step 8 adaptive-R EKF tuning params — only meaningful when
        # observer_mode=1. Sent first so the adaptive path wakes up with the
        # correct (ω_thresh, R0, Qe, vfpl) if mode=1 is selected.
        if self._observer_mode == 1:
            wthr = self._ekf_omega_thresh
            if 50 <= wthr <= 400:
                cfg_frames.append(('ekf_wthr', build_config(CFG_PARAM_EKF_WTHR, wthr)))
            r0_enc = int(round(self._ekf_r0 * 10000.0))
            if 1 <= r0_enc <= 10000:
                cfg_frames.append(('ekf_r0', build_config(CFG_PARAM_EKF_R0, r0_enc)))
            qe_enc = int(round(self._ekf_qe * 10000.0))
            if 1 <= qe_enc <= 10000:
                cfg_frames.append(('ekf_qe', build_config(CFG_PARAM_EKF_QE, qe_enc)))
            vfpl = self._ekf_vf_prior_lock
            if vfpl in (0, 1):
                cfg_frames.append(('ekf_vfpl', build_config(CFG_PARAM_EKF_VFPL, vfpl)))

        # Always send obs_mode (0 or 1). Firmware's cfg_observer_mode is a
        # static var that persists across node restarts — without an explicit
        # frame it keeps the previous session's value.
        cfg_frames.append(('obs_mode',
                           build_config(CFG_PARAM_OBS_MODE, self._observer_mode)))

        for name, frame in cfg_frames:
            try:
                self._ser.write(frame)
                time.sleep(0.01)
            except serial.SerialException as e:
                self.get_logger().error(f'Config send failed ({name}): {e}')
        if cfg_frames:
            self.get_logger().info(
                f'Config sent: max_spd={spd} RPM, iq_lim={self._iq_limit_a:.1f} A, '
                f'ol_iq={self._ol_iq_a:.1f} A, ol_ramp={self._ol_ramp_ms} ms, '
                f'align={self._align_ms} ms @ {self._align_id_a:.1f} A, '
                f'xf_dur={self._xf_duration_ms} ms, xf_dwell={self._xf_dwell_ms} ms, '
                f'ol_rpm={self._ol_target_rpm} RPM, '
                f'spd_kp={self._spd_kp}, spd_ki={self._spd_ki}, '
                f'spd_lpf={self._spd_lpf_alpha}')
        if self._observer_mode == 1:
            self.get_logger().warn(
                f'ADAPTIVE-R EKF enabled: omega_thresh={self._ekf_omega_thresh} rad/s, '
                f'R0={self._ekf_r0:.3e}, Q_e={self._ekf_qe:.3e}, '
                f'vf_prior_lock={self._ekf_vf_prior_lock}')

    # -- cmd_vel_stamped callback -----------------------------------------------

    def _cmd_vel_cb(self, msg: TwistStamped):
        vx = msg.twist.linear.x

        with self._lock:
            self._last_cmd_vel_time = time.time()

            if abs(vx) < self._deadband * self._max_linear_mps:
                self._cmd_vel_dir = 0
                self._u_target = 0.0
            else:
                self._cmd_vel_dir = 1 if vx > 0 else -1

                if self._cfoc_st == 'CLOSED_LOOP':
                    if self._cl_mode == 'full':
                        # Bumpless CL entry floor: u_bumpless keeps the PI setpoint
                        # matched to OL handover so the motor isn't braked on entry.
                        u_bumpless = float(self._ol_target_rpm) / float(self._max_speed_rpm)
                        u_bumpless = max(self._u_startup, min(self._u_max, u_bumpless))

                        # Stick magnitude picks a target inside [u_bumpless, u_max].
                        # This gives progressive throttle feel even when the speed PI
                        # is saturation-limited on ground; |vx| > max_linear_mps saturates.
                        stick_mag = min(1.0, abs(vx) / self._max_linear_mps)
                        u_stick = u_bumpless + stick_mag * (self._u_max - u_bumpless)

                        # Time-based blend from u_bumpless to u_stick over cl_post_ramp_ms
                        # keeps the bumpless entry smooth; after the ramp, stick drives u directly.
                        elapsed_ms = (time.time() - self._cl_entry_time) * 1000.0
                        ramp_ms = max(1.0, float(self._cl_post_ramp_ms))
                        k = max(0.0, min(1.0, elapsed_ms / ramp_ms))
                        u_mag = u_bumpless + k * (u_stick - u_bumpless)
                        self._u_target = u_mag * self._cmd_vel_dir
                    else:
                        # Proportional control in closed-loop
                        u = vx / self._max_linear_mps
                        self._u_target = max(-self._u_max, min(self._u_max, u))
                else:
                    # Fixed startup command — direction only
                    self._u_target = self._u_startup * self._cmd_vel_dir

    # -- Sender thread ---------------------------------------------------------

    def _sender_thread(self):
        interval = 1.0 / self._cmd_hz
        while not self._stop.is_set():
            with self._lock:
                # cmd_vel timeout → force neutral
                if (self._last_cmd_vel_time > 0.0 and
                        time.time() - self._last_cmd_vel_time > self._cmd_vel_timeout):
                    self._cmd_vel_dir = 0
                    self._u_target = 0.0

                # During startup, enforce fixed command so open-loop ramp is clean
                elif self._cmd_vel_dir != 0 and self._cfoc_st != 'CLOSED_LOOP':
                    self._u_target = self._u_startup * self._cmd_vel_dir

                delta = self._u_target - self._u_current
                delta = max(-self._slew_rate, min(self._slew_rate, delta))
                self._u_current += delta
                u_out = self._u_current

            try:
                self._ser.write(build_command(u_out))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write: {e}')
                break
            time.sleep(interval)

    # -- Reader thread ---------------------------------------------------------

    def _reader_thread(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read: {e}')
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

                (speed, esc_st, faults, u_val, vbus, iq_ma, id_ma,
                 cfoc_st, innov_a, kappa, residual,
                 swap_def, cl_hyst_ms) = result

                # Update CFOC state for control logic
                with self._lock:
                    if cfoc_st == 'CLOSED_LOOP' and self._cfoc_st != 'CLOSED_LOOP':
                        self._cl_entry_time = time.time()
                        # Bumpless u snap on CL entry (cl_mode='full' only):
                        # jump u_current to match OL handover speed so the
                        # speed PI setpoint doesn't lag and brake the motor.
                        if self._cl_mode == 'full' and self._cmd_vel_dir != 0:
                            u_bump = float(self._ol_target_rpm) / float(self._max_speed_rpm)
                            u_bump = max(self._u_startup, min(self._u_max, u_bump))
                            self._u_current = u_bump * self._cmd_vel_dir
                    self._cfoc_st = cfoc_st

                self._pub_speed.publish(Int16(data=speed))
                self._pub_state.publish(String(data=esc_st))
                self._pub_faults.publish(String(data=faults))
                self._pub_cmd.publish(Float32(data=float(u_val)))
                self._pub_vbus.publish(Float32(data=float(vbus)))
                self._pub_iq_ma.publish(Int16(data=iq_ma))
                self._pub_id_ma.publish(Int16(data=id_ma))
                self._pub_cfoc_state.publish(String(data=cfoc_st))
                self._pub_innov.publish(Float32(data=float(innov_a)))
                self._pub_kappa.publish(Float32(data=float(kappa)))
                self._pub_residual.publish(Float32(data=float(residual)))
                self._pub_swap_def.publish(Int16(data=swap_def))
                self._pub_cl_hyst.publish(Int16(data=cl_hyst_ms))

                self.get_logger().debug(
                    f'spd={speed} RPM  esc={esc_st}  cfoc={cfoc_st}  '
                    f'u={u_val:+.3f}  iq={iq_ma} mA  faults={faults}')

                buf = buf[TLM_FRAME_LEN:]

    # -- Shutdown --------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Sending neutral and closing port ...')
        with self._lock:
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
    node = EscNodeCustomFoc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
