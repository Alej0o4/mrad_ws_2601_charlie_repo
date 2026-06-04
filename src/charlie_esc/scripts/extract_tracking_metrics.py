#!/usr/bin/env python3
"""
extract_tracking_metrics.py -- Analiza múltiples bag files de ROS 2 (Jazzy) para 
                               evaluar el rendimiento de algoritmos de path tracking.

Uso:
    python3 extract_tracking_metrics.py <ruta_carpeta_padre_con_bags>
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.spatial import cKDTree
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# --- Definición del Typestore para ROS 2 Jazzy ---
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)

def extract_odometry_pose(msg):
    """Extrae únicamente x, y de nav_msgs/msg/Odometry"""
    return (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y
    )

def extract_twist_stamped(msg):
    """Extrae la velocidad lineal x de geometry_msgs/msg/TwistStamped
       Sirve tanto para /cmd_vel_raw (escalada) como para /odom/twist (real)"""
    return (msg.twist.linear.x,)

def extract_float32(msg):
    """Extrae el comando de dirección de std_msgs/msg/Float32"""
    return (msg.data,)

def read_topic(reader, topic_name, extract_fn):
    """Itera sobre un tópico y devuelve un DataFrame con [t, *campos]"""
    connections = [c for c in reader.connections if c.topic == topic_name]
    if not connections:
        return pd.DataFrame()
    
    rows = []
    for conn, timestamp, rawdata in reader.messages(connections=connections):
        msg = TYPESTORE.deserialize_cdr(rawdata, conn.msgtype)
        t_sec = timestamp * 1e-9
        rows.append((t_sec,) + extract_fn(msg))
    
    return pd.DataFrame(rows)

def extract_reference_path(reader):
    """Extrae el camino de referencia como un array Nx2 de nav_msgs/msg/Path"""
    # Actualizado a /smoothed_path
    connections = [c for c in reader.connections if c.topic == '/smoothed_path']
    if not connections:
        return None
    
    # Solo leemos el primer mensaje, ya que el path es estático
    for conn, _, rawdata in reader.messages(connections=connections):
        msg = TYPESTORE.deserialize_cdr(rawdata, conn.msgtype)
        waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        return np.array(waypoints)
    return None

def extract_slam_pose(msg):
    """Extrae la posición X, Y global y corregida directamente de SLAM"""
    return (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y
    )

def process_bag(bag_path):
    """Procesa un solo bag extrayendo la posición global real de SLAM."""
    print(f"Procesando: {bag_path.name}")
    
    with Reader(bag_path) as reader:
        # 1. Extraemos la posición real (Baja frecuencia, ~2Hz)
        df_slam = read_topic(reader, '/pose', extract_slam_pose)
        
        # 2. Extraemos los datos del vehículo (Alta frecuencia, ~30Hz+)
        df_twist = read_topic(reader, '/odom/twist', extract_twist_stamped) 
        df_cmd_vel = read_topic(reader, '/cmd_vel_raw', extract_twist_stamped)
        df_steer = read_topic(reader, '/servo/steering_cmd', extract_float32)
        ref_path = extract_reference_path(reader)

    if df_slam.empty or df_twist.empty:
        print(f"  [ERROR] Faltan datos críticos (/pose o /odom/twist) en {bag_path.name}")
        return None, None

    # Asignar columnas
    df_slam.columns = ['Timestamp', 'X', 'Y']
    df_twist.columns = ['Timestamp', 'V_real']

    # --- FUSIÓN E INTERPOLACIÓN (El núcleo de la solución) ---
    # Unimos todos los timestamps (los de SLAM y los del Twist) en una sola línea de tiempo
    df_merged = pd.merge(df_twist, df_slam, on='Timestamp', how='outer')
    df_merged.sort_values('Timestamp', inplace=True)
    df_merged.reset_index(drop=True, inplace=True)

    # Interpolamos linealmente X e Y para rellenar los milisegundos vacíos
    df_merged['X'] = df_merged['X'].interpolate(method='linear')
    df_merged['Y'] = df_merged['Y'].interpolate(method='linear')

    # [CORRECCIÓN]: Propagar el primer y último valor de SLAM a los bordes 
    # por si la odometría arrancó antes o terminó después que el SLAM
    df_merged['X'] = df_merged['X'].bfill().ffill()
    df_merged['Y'] = df_merged['Y'].bfill().ffill()

    # Eliminamos las filas donde V_real es NaN
    df_merged = df_merged.dropna(subset=['V_real'])
    
    # [SEGURIDAD EXTREMA]: Garantizar que no haya ningún NaN residual en X o Y 
    # antes de pasarle la matriz al cKDTree de Scipy
    df_merged = df_merged.dropna(subset=['X', 'Y'])

    # --- Sincronizar el resto de comandos ---
    if not df_cmd_vel.empty:
        df_cmd_vel.columns = ['Timestamp', 'V_cmd_scaled']
        df_cmd_vel.sort_values('Timestamp', inplace=True)
        df_merged = pd.merge_asof(df_merged, df_cmd_vel, on='Timestamp', direction='backward')
    else:
        df_merged['V_cmd_scaled'] = np.nan

    if not df_steer.empty:
        df_steer.columns = ['Timestamp', 'Steering']
        df_steer.sort_values('Timestamp', inplace=True)
        df_merged = pd.merge_asof(df_merged, df_steer, on='Timestamp', direction='backward')
    else:
        df_merged['Steering'] = np.nan

    # Normalizar Timestamp a t=0
    df_merged['Timestamp'] -= df_merged['Timestamp'].iloc[0]

    # --- Cálculo del Cross-Track Error (CTE) ---
    if ref_path is not None and len(ref_path) > 0:
        kdtree = cKDTree(ref_path)
        robot_positions = df_merged[['X', 'Y']].values
        distances, _ = kdtree.query(robot_positions)
        df_merged['CTE'] = distances
        
        # Guardar el path de referencia para Colab (se sobrescribirá, está bien)
        pd.DataFrame(ref_path, columns=['X', 'Y']).to_csv("metricas_exportadas/reference_path.csv", index=False)
    else:
        print("  [WARNING] No se encontró /smoothed_path. CTE será NaN.")
        df_merged['CTE'] = np.nan

    # --- Cálculo de Métricas Finales ---
    t_total = df_merged['Timestamp'].iloc[-1]
    dx = np.diff(df_merged['X'])
    dy = np.diff(df_merged['Y'])
    dist_total = np.sum(np.sqrt(dx**2 + dy**2))
    v_avg = df_merged['V_real'].mean()
    v_max = df_merged['V_real'].max()
    cte_rmse = np.sqrt(np.mean(df_merged['CTE']**2)) if 'CTE' in df_merged else np.nan
    cte_max = df_merged['CTE'].max() if 'CTE' in df_merged else np.nan
    
    if 'Steering' in df_merged and not df_merged['Steering'].isna().all():
        dt = np.diff(df_merged['Timestamp'])
        d_steer = np.diff(df_merged['Steering'])
        d_steer_dt = np.divide(d_steer, dt, out=np.zeros_like(d_steer), where=dt!=0)
        control_effort = np.var(d_steer_dt)
    else:
        control_effort = np.nan

    metrics = {
        'Bag_Name': bag_path.name,
        'Tiempo_Total_s': round(t_total, 3),
        'Distancia_m': round(dist_total, 3),
        'V_Real_Avg_ms': round(v_avg, 3),
        'V_Real_Max_ms': round(v_max, 3),
        'CTE_RMSE_m': round(cte_rmse, 4),
        'CTE_Max_m': round(cte_max, 4),
        'Control_Effort_Var': round(control_effort, 4)
    }

    cols_order = ['Timestamp', 'X', 'Y', 'V_cmd_scaled', 'V_real', 'Steering', 'CTE']
    df_merged = df_merged[[c for c in cols_order if c in df_merged.columns]]

    return metrics, df_merged

def main():
    if len(sys.argv) < 2:
        print("Uso: python3 extract_tracking_metrics.py <carpeta_padre_de_bags>")
        sys.exit(1)

    base_dir = Path(sys.argv[1])
    if not base_dir.exists() or not base_dir.is_dir():
        print(f"Error: La ruta {base_dir} no es válida.")
        sys.exit(1)

    output_dir = Path("metricas_exportadas")
    output_dir.mkdir(exist_ok=True)

    summary_metrics = []

    # Iterar sobre las subcarpetas
    for bag_path in base_dir.iterdir():
        if bag_path.is_dir():
            # Validación sencilla: revisar si existe metadata.yaml
            if (bag_path / "metadata.yaml").exists():
                metrics, df_ts = process_bag(bag_path)
                
                if metrics and df_ts is not None:
                    summary_metrics.append(metrics)
                    
                    # Guardar serie de tiempo detallada
                    ts_filename = output_dir / f"{bag_path.name}_timeseries.csv"
                    df_ts.to_csv(ts_filename, index=False)
                    print(f"  -> Guardado {ts_filename.name}")

    if summary_metrics:
        df_summary = pd.DataFrame(summary_metrics)
        summary_path = output_dir / "resultados_tracking.csv"
        df_summary.to_csv(summary_path, index=False)
        print(f"\nExtracción completada. Resumen general guardado en: {summary_path}")
        print(df_summary.to_string())
    else:
        print("\nNo se encontraron datos válidos para procesar.")

if __name__ == '__main__':
    main()