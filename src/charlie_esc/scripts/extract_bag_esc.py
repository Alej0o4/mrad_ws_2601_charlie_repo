#!/usr/bin/env python3
"""
extract_bag_sysid.py  --  Read a rosbag and export synchronized data to CSV,
                          including custom ESC telemetry for System Identification.

Usage:
    python3 extract_bag_sysid.py <path_to_bag_folder>
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

if len(sys.argv) < 2:
    print("Uso: python3 extract_bag_sysid.py <ruta_a_carpeta_bag>")
    sys.exit(1)

BAG_PATH = Path(sys.argv[1])
BAG_NAME = BAG_PATH.name
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)

OUTPUT_DIR = Path("csv_files")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

def read_topic(reader, typestore, topic_name, fields_fn):
    """Extract (timestamp_s, *fields) from a topic."""
    rows = []
    connections = [c for c in reader.connections if c.topic == topic_name]
    if not connections:
        print(f"  [WARNING] topic '{topic_name}' not found in bag")
        return pd.DataFrame()
    for conn, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        t = timestamp * 1e-9  # nanoseconds → seconds
        rows.append((t,) + fields_fn(msg))
    return pd.DataFrame(rows)

# --- Funciones de Extracción ---

def extract_pose(msg):
    """Extrae datos del mensaje geometry_msgs/msg/Pose (VICON)"""
    p = msg.position
    o = msg.orientation
    # Asumiendo movimiento en 2D para el yaw
    yaw = np.arctan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    )
    return (p.x, p.y, p.z, yaw)

def extract_cmd_vel(msg):
    """Extrae datos del mensaje geometry_msgs/msg/TwistStamped"""
    return (float(msg.twist.linear.x), float(msg.twist.angular.z))

# Extractores para std_msgs (Telemetría del ESC)
def extract_std_float(msg):
    return (float(msg.data),)

def extract_std_int(msg):
    return (int(msg.data),)

def extract_std_string(msg):
    return (str(msg.data),)

# --- Lectura del Bag ---

with Reader(BAG_PATH) as reader:
    print(f"Reading bag: {BAG_PATH}")

    # Ground Truth y Referencia
    df_pose = read_topic(reader, TYPESTORE, "/robot1/pose", extract_pose)
    df_cmd_ref = read_topic(reader, TYPESTORE, "/cmd_vel_stamped", extract_cmd_vel)
    
    # Telemetría del ESC (Crucial para SysId)
    df_esc_cmd = read_topic(reader, TYPESTORE, "/esc/command", extract_std_float)
    df_esc_state = read_topic(reader, TYPESTORE, "/esc/cfoc_state", extract_std_string)
    df_esc_iq = read_topic(reader, TYPESTORE, "/esc/iq_ma", extract_std_int)
    df_esc_rpm = read_topic(reader, TYPESTORE, "/esc/speed_rpm", extract_std_int)

# --- Asignación de Columnas ---

if not df_pose.empty: df_pose.columns = ["t", "x", "y", "z", "yaw"]
if not df_cmd_ref.empty: df_cmd_ref.columns = ["t", "cmd_linear_x", "cmd_angular_z"]
if not df_esc_cmd.empty: df_esc_cmd.columns = ["t", "esc_command"]
if not df_esc_state.empty: df_esc_state.columns = ["t", "cfoc_state"]
if not df_esc_iq.empty: df_esc_iq.columns = ["t", "iq_ma"]
if not df_esc_rpm.empty: df_esc_rpm.columns = ["t", "speed_rpm"]

# --- Alineación de Tiempos ---

dataframes = {
    "pose": df_pose, 
    "cmd_ref": df_cmd_ref,
    "esc_cmd": df_esc_cmd,
    "esc_state": df_esc_state,
    "esc_iq": df_esc_iq,
    "esc_rpm": df_esc_rpm
}

valid_dfs = []

for name, df in dataframes.items():
    if not df.empty:
        df.sort_values("t", inplace=True)
        df.reset_index(drop=True, inplace=True)
        valid_dfs.append(df)

if valid_dfs:
    # Alinear tiempo para que empiece en 0 de forma segura
    t0 = min(df["t"].iloc[0] for df in valid_dfs)
    for df in valid_dfs:
        df["t"] -= t0

# --- Guardado de Archivos ---

EXPERIMENT_DIR = OUTPUT_DIR / BAG_NAME
EXPERIMENT_DIR.mkdir(parents=True, exist_ok=True)

for name, df in dataframes.items():
    if not df.empty:
        path = EXPERIMENT_DIR / f"{name}.csv"
        df.to_csv(path, index=False)
        dt_ms = df['t'].diff().median() * 1000
        print(f"  {name.upper():<10}: {len(df):>5} samples, dt={dt_ms:>5.1f} ms -> {path}")

print("Extracción completada con éxito.")