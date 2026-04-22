#!/usr/bin/env python3
"""
extract_bag.py  --  Read a rosbag and export synchronized data to CSV.

Usage:
    python3 extract_bag.py <path_to_bag_folder>
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# Validar argumentos
if len(sys.argv) < 2:
    print("Uso: python3 extract_bag.py <ruta_a_carpeta_bag>")
    sys.exit(1)

BAG_PATH = Path(sys.argv[1])
BAG_NAME = BAG_PATH.name # Extrae el nombre de la bag (ej. sysid_run_adelante_1)
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)

# Definir y crear carpeta de salida
OUTPUT_DIR = Path("csv_files")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

def read_topic(reader, typestore, topic_name, fields_fn):
    """Extract (timestamp_s, *fields) from a topic using fields_fn(msg) -> tuple."""
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


def extract_odom(msg):
    """Extrae datos del mensaje nav_msgs/msg/Odometry"""
    # En Odometry, la pose está dentro de pose.pose
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    
    # Convert quaternion to yaw (2D assumption)
    yaw = np.arctan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    )
    return (p.x, p.y, p.z, yaw)


def extract_cmd_vel(msg):
    """Extrae datos del mensaje geometry_msgs/msg/TwistStamped"""
    # Extraemos velocidad lineal en X y angular en Z
    linear_x = float(msg.twist.linear.x)
    angular_z = float(msg.twist.angular.z)
    return (linear_x, angular_z)


def extract_joy(msg):
    """Extrae datos del mensaje sensor_msgs/msg/Joy"""
    # axes[1] = left stick vertical (throttle), axes[3] = right stick horizontal (steering)
    throttle = float(msg.axes[1]) if len(msg.axes) > 1 else 0.0
    steering  = float(msg.axes[3]) if len(msg.axes) > 3 else 0.0
    return (throttle, steering)


with Reader(BAG_PATH) as reader:
    print(f"Reading bag: {BAG_PATH}")

    # Tópicos actualizados según el ros2 bag info
    df_odom = read_topic(reader, TYPESTORE, "/odom", extract_odom)
    df_cmd  = read_topic(reader, TYPESTORE, "/cmd_vel_raw", extract_cmd_vel)
    df_joy  = read_topic(reader, TYPESTORE, "/joy", extract_joy)

# Asignar nombres a las columnas si los DataFrames no están vacíos
if not df_odom.empty:
    df_odom.columns = ["t", "x", "y", "z", "yaw"]
if not df_cmd.empty:
    df_cmd.columns  = ["t", "cmd_linear_x", "cmd_angular_z"]
if not df_joy.empty:
    df_joy.columns  = ["t", "joy_throttle", "joy_steering"]

# Ordenar por tiempo y resetear índices
dataframes = {"odom": df_odom, "cmd": df_cmd, "joy": df_joy}
valid_dfs = []

for name, df in dataframes.items():
    if not df.empty:
        df.sort_values("t", inplace=True)
        df.reset_index(drop=True, inplace=True)
        valid_dfs.append(df)

# Alinear tiempo para que empiece en 0 de forma segura
if valid_dfs:
    # Encuentra el tiempo mínimo global entre todos los dataframes válidos
    t0 = min(df["t"].iloc[0] for df in valid_dfs)
    
    for df in valid_dfs:
        df["t"] -= t0


EXPERIMENT_DIR = OUTPUT_DIR / BAG_NAME
EXPERIMENT_DIR.mkdir(parents=True, exist_ok=True)
# Guardar los archivos en la carpeta csv_files con el nombre de la bag como prefijo
if not df_odom.empty:
    path_odom = EXPERIMENT_DIR / "odom.csv"
    df_odom.to_csv(path_odom, index=False)
    print(f"  ODOM   : {len(df_odom)} samples, dt={df_odom['t'].diff().median()*1000:.1f} ms -> {path_odom}")

if not df_cmd.empty:
    path_cmd = EXPERIMENT_DIR / "cmd.csv"
    df_cmd.to_csv(path_cmd, index=False)
    print(f"  CMD_VEL: {len(df_cmd)} samples, dt={df_cmd['t'].diff().median()*1000:.1f} ms -> {path_cmd}")

if not df_joy.empty:
    path_joy = EXPERIMENT_DIR / "joy.csv"
    df_joy.to_csv(path_joy, index=False)
    print(f"  JOY    : {len(df_joy)} samples, dt={df_joy['t'].diff().median()*1000:.1f} ms -> {path_joy}")

print("Extracción completada.")