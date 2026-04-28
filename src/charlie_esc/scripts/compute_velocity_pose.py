#!/usr/bin/env python3
"""
compute_velocity.py  --  Derive v(t) and omega(t) from VICON pose data.
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import savgol_filter

if len(sys.argv) < 2:
    print("Uso: python3 compute_velocity.py <ruta_a_carpeta_experimento>")
    sys.exit(1)

EXPERIMENT_DIR = Path(sys.argv[1])
POSE_FILE = EXPERIMENT_DIR / "pose.csv" # Cambiado a la data del VICON

if not POSE_FILE.exists():
    print(f"[ERROR] No se encontró el archivo: {POSE_FILE}")
    sys.exit(1)

df = pd.read_csv(POSE_FILE)
t   = df["t"].values
x   = df["x"].values
y   = df["y"].values
yaw = np.unwrap(df["yaw"].values)

# --- Savitzky-Golay differentiation ---
# Nota: Como el VICON va a ~100Hz (según los 2099 msgs en 21s), 
# podrías necesitar ajustar el WINDOW si ves que la señal queda muy ruidosa.
WINDOW  = 21    
POLY    = 3     

dt = np.median(np.diff(t))

vx    = savgol_filter(x,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
vy    = savgol_filter(y,   window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)
omega = savgol_filter(yaw, window_length=WINDOW, polyorder=POLY, deriv=1, delta=dt)

v_signed = vx * np.cos(yaw) + vy * np.sin(yaw)

df_out = pd.DataFrame({
    "t": t, "x": x, "y": y, "yaw": yaw,
    "vx": vx, "vy": vy, "v": v_signed, "omega": omega
})

OUTPUT_FILE = EXPERIMENT_DIR / "vicon_velocity.csv"
df_out.to_csv(OUTPUT_FILE, index=False)

print(f"Saved: {OUTPUT_FILE}")
print(f"  v range : [{v_signed.min():.3f}, {v_signed.max():.3f}] m/s")
print(f"  ω range : [{omega.min():.3f}, {omega.max():.3f}] rad/s")