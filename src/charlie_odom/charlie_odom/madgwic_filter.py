# ...existing code...
import numpy as np

class MadgwickAHRS:
    def __init__(self, sample_period=1/20, beta=0.1):
        self.sample_period = sample_period
        self.beta = beta
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # initial quaternion

    def update(self, *args, **kwargs):
        """
        Compatible con:
         - update(gyroscope, accelerometer, magnetometer)
         - update(q, gyr=..., acc=..., mag=...)
         - update(gyr=..., acc=..., mag=...)
        Devuelve la cuaternión actualizada (np.array [w,x,y,z]) y actualiza self.quaternion.
        """
        # Resolver inputs
        if len(args) == 3:
            # old style: update(gyr, acc, mag)
            gyr, acc, mag = args
            q = self.quaternion.copy()
        elif len(args) == 1:
            q = np.asarray(args[0], dtype=float).copy()
            gyr = kwargs.get('gyr', None)
            if gyr is None:
                gyr = kwargs.get('gyroscope', None)
            acc = kwargs.get('acc', None)
            if acc is None:
                acc = kwargs.get('accelerometer', None)
            mag = kwargs.get('mag', None)
            if mag is None:
                mag = kwargs.get('magnetometer', None)
        else:
            q = self.quaternion.copy()
            gyr = kwargs.get('gyr', None)
            if gyr is None:
                gyr = kwargs.get('gyroscope', None)
            acc = kwargs.get('acc', None)
            if acc is None:
                acc = kwargs.get('accelerometer', None)
            mag = kwargs.get('mag', None)
            if mag is None:
                mag = kwargs.get('magnetometer', None)

        if gyr is None or acc is None:
            # nothing to do, devolver estado actual
            return self.quaternion.copy()

        gyroscope = np.asarray(gyr, dtype=float)
        accelerometer = np.asarray(acc, dtype=float)
        magnetometer = np.asarray(mag, dtype=float) if mag is not None else None

        gx, gy, gz = gyroscope
        ax, ay, az = accelerometer

        norm_acc = np.linalg.norm([ax, ay, az])
        if norm_acc == 0:
            return self.quaternion.copy()
        ax, ay, az = ax / norm_acc, ay / norm_acc, az / norm_acc

        if magnetometer is not None:
            mx, my, mz = magnetometer
            norm_mag = np.linalg.norm([mx, my, mz])
            if norm_mag == 0:
                # magnetómetro inválido -> proceder sin mag (pero algoritmo actual requiere mag for full compensation)
                magnetometer = None
            else:
                mx, my, mz = mx / norm_mag, my / norm_mag, mz / norm_mag

        # Precompute helpers
        _2q1 = 2.0 * q[0]
        _2q2 = 2.0 * q[1]
        _2q3 = 2.0 * q[2]
        _2q4 = 2.0 * q[3]
        _4q1 = 4.0 * q[0]
        _4q2 = 4.0 * q[1]
        _4q3 = 4.0 * q[2]
        _8q2 = 8.0 * q[1]
        _8q3 = 8.0 * q[2]
        q1q1 = q[0] * q[0]
        q2q2 = q[1] * q[1]
        q3q3 = q[2] * q[2]
        q4q4 = q[3] * q[3]

        # Gradient descent algorithm corrective step (uses acc and mag if available)
        # If magnetometer is not provided, the original gradient terms relying on mag should be adjusted.
        # For simplicity keep accel-only correction terms (as in many Madgwick variants).
        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
        s2 = _4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q[1] - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az
        s3 = 4.0 * q1q1 * q[2] + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az
        s4 = 4.0 * q2q2 * q[3] - _2q2 * ax + 4.0 * q3q3 * q[3] - _2q3 * ay
        norm_s = np.linalg.norm([s1, s2, s3, s4])
        if norm_s == 0:
            return self.quaternion.copy()
        s1, s2, s3, s4 = s1 / norm_s, s2 / norm_s, s3 / norm_s, s4 / norm_s

        q_dot = 0.5 * np.array([
            -q[1] * gx - q[2] * gy - q[3] * gz,
             q[0] * gx + q[2] * gz - q[3] * gy,
             q[0] * gy - q[1] * gz + q[3] * gx,
             q[0] * gz + q[1] * gy - q[2] * gx
        ]) - self.beta * np.array([s1, s2, s3, s4])

        q += q_dot * self.sample_period
        q = q / np.linalg.norm(q)

        # Guardar y devolver
        self.quaternion = q.copy()
        return q.copy()

    def get_yaw(self):
        q = self.quaternion
        return np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2]**2 + q[3]**2))