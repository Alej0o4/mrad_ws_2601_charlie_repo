#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from dataclasses import dataclass

@dataclass
class LineModel:
    points: list        # Lista de coordenadas [x, y] que pertenecen a la línea
    a: float = 0.0      # Parámetro de la ecuación ax+by+c=0
    b: float = 0.0
    c: float = 0.0
    start_point: tuple = (0.0, 0.0) # Vértice proyectado inicial
    end_point: tuple = (0.0, 0.0)   # Vértice proyectado final
    
    # --- Nuevos atributos agregados para la lógica de solapamiento ---
    start_idx: int = -1             # Índice de inicio del segmento en el arreglo del LiDAR
    end_idx: int = -1               # Índice de fin del segmento en el arreglo del LiDAR

class LineSegment(Node):
    """
    Line segment extraction node.
    Subscribes:
        - /scan (sensor_msgs/LaserScan)
    Publishes:
      - /extracted_lines visualization (visualization_msgs/MarkerArray)
    """

    def __init__(self):
        super().__init__('line_segment_extraction_node')
        # ---- Parameters ----
        self.declare_parameter('epsilon', 0.05)  # Distancia máxima para considerar puntos como parte de la misma línea
        self.declare_parameter('s_num', 6)   # Número mínimo de puntos para plantar una semilla de una línea
        self.declare_parameter('p_min', 10)  # Distancia mínima entre puntos para ser considerados válidos
        self.declare_parameter('line_width', 0.02)  # Ancho de las líneas para visualización
        self.declare_parameter('l_min', 0.1)  # Longitud mínima de una línea para ser considerada válida
        self.declare_parameter('delta', 0.05)  # Distancia máxima para considerar que un punto pertenece a una línea durante el crecimiento

        # Carga inicial de parámetros
        self.epsilon = self.get_parameter('epsilon').value
        self.s_num = self.get_parameter('s_num').value
        self.p_min = self.get_parameter('p_min').value
        self.line_width = self.get_parameter('line_width').value
        self.l_min = self.get_parameter('l_min').value
        self.delta = self.get_parameter('delta').value

        # Qos para suscripciones y publicaciones
        qos_sensor = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(depth=10)

        # Suscripción al Lidar
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)

        # Publicación de líneas extraídas
        self.lines_pub = self.create_publisher(MarkerArray, '/extracted_lines', qos_reliable)

        self.get_logger().info("Line Segment Extraction Node Ready")
        self.get_logger().info(f"Parameters: epsilon={self.epsilon:.2f}, s_num={self.s_num}, p_min={self.p_min:.2f}, line_width={self.line_width:.2f}, l_min={self.l_min:.2f}")


    ### ===== Callbacks ===== ###
    def scan_callback(self, msg):
        """
        Callback principal. Orquesta el pipeline de extracción de líneas.
        """
        # 1. Módulo de Transformación (Pre-procesamiento)
        # Convertimos los rangos polares a (x, y) ignorando valores infinitos
        points = self.polar_to_cartesian(msg)
        
        # Si el LiDAR no detectó casi nada, abortamos
        if len(points) < self.p_min:
            return

        extracted_segments = []
        current_idx = 0
        num_points = len(points)

        # 2. Pipeline de Extracción (Módulos de Semilla y Crecimiento)
        # Barremos el arreglo de puntos de inicio a fin
        while current_idx <= num_points - self.p_min:
            
            # Pasamos un sub-arreglo a la función de detección
            sub_points = points[current_idx:]
            
            # Buscamos la primera semilla válida
            seed_model, rel_start, rel_end = self.detection_seed_segments(sub_points)
            
            if seed_model is None:
                # Si ya no hay más líneas rectas en lo que queda del escaneo, terminamos
                break 
                
            # Convertimos los índices relativos del sub-arreglo a índices absolutos
            abs_start = current_idx + rel_start
            abs_end = current_idx + rel_end
            
            # Crecemos la región a partir de la semilla
            grown_line, pb, pf = self.region_growing(points, seed_model, abs_start, abs_end)
            
            if grown_line is not None:
                # ¡Línea exitosa! Guardamos sus índices finales en el objeto 
                # (útil para el solapamiento) y la agregamos a nuestra lista.
                grown_line.start_idx = pb
                grown_line.end_idx = pf
                extracted_segments.append(grown_line)
                
                # Avanzamos el puntero justo después de donde terminó esta línea 
                # para no volver a evaluar los puntos que ya fueron absorbidos.
                current_idx = pf + 1
            else:
                # Si la semilla fue válida pero falló al crecer (ej. era muy corta),
                # avanzamos el puntero un poco para seguir buscando y no atascarnos.
                current_idx = abs_start + 1

        # 3. Módulo de Procesamiento de Solapamiento
        # Resolvemos colisiones, unimos líneas colineales y refinamos las esquinas.
        final_segments = self.overlap_region_processing(extracted_segments)
        
        # 4. Módulo de Proyección de Endpoints
        # (Opcional, pero recomendado por el profe) Ajustamos matemáticamente los vértices
        for line in final_segments:
            # Asumimos que el primer y último punto de la lista son los extremos "crudos"
            raw_start = line.points[0]
            raw_end = line.points[-1]
            line.start_point, line.end_point = self.calculate_endpoints(
                raw_start, raw_end, line.a, line.b, line.c
            )

        # 5. Visualización
        # Publicamos los segmentos a RViz
        if final_segments:
            self.publish_extracted_lines(final_segments)  

    ### ===== Funciones para extracción de líneas ===== ###
    def detection_seed_segments(self, points):
        """
        Busca la primera semilla válida en un conjunto de puntos cartesianos 2D.
        
        Input: 
            points: Lista de tuplas o arrays [(x1,y1), (x2,y2), ... (xN,yN)]
        Output: 
            seed_model: Objeto LineModel instanciado si encuentra una semilla.
            start_idx: Índice donde empieza la semilla.
            end_idx: Índice donde termina la semilla.
            (Si no encuentra nada, retorna None, -1, -1)
        """
        num_points = len(points)
        
        # Diferencia 1: Si los puntos restantes son menores que el mínimo exigido 
        # para una línea completa (Pmin), es matemáticamente imposible formar una línea.
        if num_points < self.p_min:
            return None, -1, -1

        # Bucle 2: for i = 1 -> (Np - Pmin)  [Adaptado a índices base 0 de Python]
        for i in range(num_points - self.p_min + 1):
            
            # Línea 3: j = i + Snum
            j = i + self.s_num
            seed_points = points[i:j]
            
            # Línea 4: fit Seed(i, j)
            a, b, c = self.fit_line_orthogonal(seed_points)
            
            # Línea 1 Corregida: Inicializamos la bandera DENTRO del bucle
            is_valid_seed = True 
            
            # Bucle 5: for k = i -> j do
            for k in range(i, j):
                pt = points[k]
                
                # Línea 6: obtain the predicted point P'_k
                # Truco de Ingeniería: Obtenemos theta usando atan2 dado que 
                # el LiDAR es el origen (0,0) en el marco de referencia base_laser.
                theta = math.atan2(pt[1], pt[0])
                pt_pred = self.predict_point_on_line(theta, a, b, c)
                
                # Líneas 7-8: d1 <- distance from P_k to P'_k
                # (Nota: en tu OCR salió un '8', pero en el paper es delta)
                d1 = self.distance_point_to_point(pt, pt_pred)
                if d1 > self.delta:
                    is_valid_seed = False
                    break
                    
                # Líneas 12-13: d2 <- distance from P_k to Seed(i, j)
                # (Nota: en tu OCR salió una 'e', pero es epsilon)
                d2 = self.distance_point_to_line(pt, a, b, c)
                if d2 > self.epsilon:
                    is_valid_seed = False
                    break
                    
            # Líneas 18-20: if flag == true then return Seed
            if is_valid_seed:
                seed_model = LineModel(
                    points=seed_points,
                    a=a, 
                    b=b, 
                    c=c
                )
                return seed_model, i, j - 1
                
        # Si termina el bucle y no encuentra nada
        return None, -1, -1

    def region_growing(self, points, seed_model, start_idx, end_idx):
        """
        Expande una semilla válida hacia adelante y hacia atrás para formar un segmento completo.
        
        Input:
            points: Lista completa de puntos del escaneo.
            seed_model: Objeto LineModel que contiene la semilla inicial.
            start_idx, end_idx: Índices en el arreglo 'points' donde inicia y termina la semilla.
        Output:
            grown_line: Objeto LineModel con la línea final expandida.
            Pb, Pf: Índices finales de inicio y fin de la línea extraída.
        """
        num_points = len(points)
        
        # 1. Inicialización (Línea 1-2 del pseudocódigo)
        # Hacemos una copia de los puntos de la semilla para no modificar la original
        current_points = list(seed_model.points)
        a, b, c = seed_model.a, seed_model.b, seed_model.c
        
        # Pf (Point Forward) apunta al siguiente punto después de la semilla
        Pf = end_idx + 1

        # 2. Crecimiento hacia adelante (Líneas 3-11)
        while Pf < num_points:
            pt = points[Pf]
            dist = self.distance_point_to_line(pt, a, b, c)
            
            # Si la distancia supera epsilon, chocamos con algo (esquina, ruido, etc.)
            if dist > self.epsilon:
                break
                
            # Línea 7: refit Line. Absorbemos el punto y recalculamos la matemática
            current_points.append(pt)
            a, b, c = self.fit_line_orthogonal(current_points)
            
            # Línea 9: Avanzamos al siguiente punto
            Pf += 1
            
        # Línea 11: Como el bucle rompió en un punto inválido, retrocedemos uno
        # para quedarnos con el último índice válido.
        Pf -= 1 

        # 3. Crecimiento hacia atrás (Líneas 12-20)
        # Pb (Point Backward) apunta al punto inmediatamente anterior a la semilla
        Pb = start_idx - 1
        
        while Pb >= 0:
            pt = points[Pb]
            dist = self.distance_point_to_line(pt, a, b, c)
            
            if dist > self.epsilon:
                break
                
            # Insertamos en la posición 0 para mantener el orden espacial del láser
            # Esto es vital para que al dibujar en RViz la línea no se cruce sobre sí misma
            current_points.insert(0, pt) 
            a, b, c = self.fit_line_orthogonal(current_points)
            
            Pb -= 1
            
        # Línea 20: Ajustamos el índice al último válido
        Pb += 1

        # 4. Validación Final (Líneas 21-24)
        # Pr (Point count): Cantidad total de puntos absorbidos
        Pr = len(current_points)
        
        # Ll (Line length): Distancia euclidiana entre el primer y último punto físico
        Ll = self.distance_point_to_point(current_points[0], current_points[-1])
        
        # Línea 22: Filtro de ruido. ¿Es una pared real o solo una basurita geométrica?
        if Ll >= self.l_min and Pr >= self.p_min:
            
            # Línea 23: Retornamos la línea consolidada
            grown_line = LineModel(
                points=current_points,
                a=a,
                b=b,
                c=c
            )
            return grown_line, Pb, Pf
            
        # Si la línea no cumple con los tamaños mínimos, devolvemos un fallo
        return None, -1, -1
    
    def overlap_region_processing(self, segments):
        """
        Procesa la lista de segmentos extraídos para resolver solapamientos,
        aplicando fusión colineal (Profesor) o resolución de esquinas (Gao et al.).
        
        Input: Lista de objetos LineModel ordenados espacialmente.
        Output: Lista final de LineModels sin solapamientos.
        """
        if len(segments) < 2:
            return segments

        final_segments = []
        
        # Tomamos el primer segmento como nuestra línea "actual" de referencia
        line_i = segments[0]

        for i in range(1, len(segments)):
            line_j = segments[i]
            
            # Condición de Solapamiento (Línea 5 del paper: m2 <= n1)
            # line_i termina en n1. line_j empieza en m2.
            if line_j.start_idx <= line_i.end_idx:
                
                # ¡APORTE DEL PROFESOR! Evaluar colinealidad primero
                if self.are_lines_collinear(line_i.a, line_i.b, line_j.a, line_j.b):
                    
                    # CASO 1: Fusión total
                    # Combinamos todos los puntos en un solo conjunto usando set 
                    # para evitar duplicados en la zona de solapamiento.
                    merged_points = []
                    # (Lógica simplificada de fusión de puntos manteniendo el orden)
                    merged_points.extend(line_i.points)
                    puntos_faltantes = line_j.points[(line_i.end_idx - line_j.start_idx + 1):]
                    merged_points.extend(puntos_faltantes)
                    
                    # Refit y actualizamos line_i para que crezca
                    a, b, c = self.fit_line_orthogonal(merged_points)
                    line_i = LineModel(
                        points=merged_points,
                        a=a, b=b, c=c,
                        start_idx=line_i.start_idx,
                        end_idx=line_j.end_idx
                    )
                    continue # Saltamos a revisar el siguiente segmento
                    
                else:
                    # CASO 2: Resolución de esquinas (Líneas 6-17 del paper)
                    # Encontramos la región de puntos compartidos
                    conflict_start = line_j.start_idx
                    conflict_end = line_i.end_idx
                    
                    # Índices de corte k
                    k_cut = conflict_end
                    
                    # Iteramos sobre los puntos en la zona de conflicto
                    # (Para hacerlo directo sobre la lista de puntos de line_i)
                    idx_offset = conflict_start - line_i.start_idx
                    
                    for k in range(conflict_start, conflict_end + 1):
                        pt_k = line_i.points[idx_offset + (k - conflict_start)]
                        
                        # Líneas 7-15 del paper: Comparar distancias
                        winner = self.resolve_corner_conflict(
                            pt_k, 
                            line_i.a, line_i.b, line_i.c,
                            line_j.a, line_j.b, line_j.c
                        )
                        
                        if winner == 2:
                            # El punto está más cerca de la Línea J. Aquí cortamos.
                            k_cut = k
                            break
                            
                    # Cortamos físicamente las listas de puntos (Líneas 14-15 del paper)
                    # La línea I se queda hasta k_cut - 1
                    line_i.points = line_i.points[:(k_cut - line_i.start_idx)]
                    line_i.end_idx = k_cut - 1
                    
                    # La línea J se queda desde k_cut en adelante
                    line_j.points = line_j.points[(k_cut - line_j.start_idx):]
                    line_j.start_idx = k_cut
                    
                    # Líneas 20-21: Refit de ambas líneas
                    line_i.a, line_i.b, line_i.c = self.fit_line_orthogonal(line_i.points)
                    line_j.a, line_j.b, line_j.c = self.fit_line_orthogonal(line_j.points)

            # Si no hay solapamiento (o ya resolvimos el conflicto de la esquina), 
            # line_i ya está terminada de forma segura. La guardamos.
            final_segments.append(line_i)
            # La línea j pasa a ser la nueva línea de referencia
            line_i = line_j
            
        # Agregamos el último segmento que quedó en memoria
        final_segments.append(line_i)
        
        return final_segments

    ### ===== Funciones para geometría ===== ###
    def fit_line_orthogonal(self, points):
        """
        Calcula los parámetros (a, b, c) de la línea ax+by+c=0 usando 
        Mínimos Cuadrados Ortogonales (PCA / Eigenvectors).
        
        Input: points (Lista o np.ndarray de puntos [[x1,y1], [x2,y2]...])
        Output: a, b, c (Floats normalizados donde a^2 + b^2 = 1)
        """
        # Convertimos a arreglo de NumPy por si acaso viene como lista
        pts = np.array(points)
        
        # Seguridad: Si hay menos de 2 puntos, no podemos formar una línea
        if len(pts) < 2:
            return 0.0, 0.0, 0.0

        # 1. Calcular el centroide (Media de X y Media de Y)
        # axis=0 significa que promediamos a lo largo de las columnas
        centroid = np.mean(pts, axis=0)

        # 2. Centrar los puntos (Restar el centroide a cada punto)
        # NumPy hace esto automáticamente para toda la matriz mediante "broadcasting"
        centered_pts = pts - centroid

        # 3. Calcular la matriz de covarianza (o matriz de dispersión)
        # rowvar=False indica que cada columna es una variable (X e Y)
        cov_matrix = np.cov(centered_pts, rowvar=False)

        # 4. Calcular Valores Propios (Eigenvalues) y Vectores Propios (Eigenvectors)
        # np.linalg.eigh está optimizada para matrices simétricas como la de covarianza.
        # Retorna los valores propios ordenados de menor a mayor.
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # 5. Extraer el vector normal (a, b)
        # El vector propio correspondiente al MENOR valor propio (índice 0) 
        # nos da directamente 'a' y 'b'.
        a, b = eigenvectors[:, 0]

        # 6. Calcular 'c' usando el centroide
        c = -(a * centroid[0] + b * centroid[1])

        return float(a), float(b), float(c)


    def distance_point_to_point(self, p1, p2):
        """
        Calcula la distancia Euclidiana entre dos puntos (x, y).
        """
        # math.hypot es más rápido y seguro contra desbordamientos que (dx**2 + dy**2)**0.5
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def distance_point_to_line(self, point, a, b, c):
        """
        Calcula la distancia perpendicular de un punto a la línea ax+by+c=0.
        """
        x0, y0 = point
        
        # Matemáticamente d = |ax0 + by0 + c| / sqrt(a^2 + b^2).
        # Aunque nuestro Mínimos Cuadrados normaliza a y b (sqrt(a^2+b^2) = 1), 
        # dejamos el denominador por pura seguridad de software.
        norm = math.hypot(a, b)
        if norm < 1e-6:
            return float('inf') # Evitar división por cero si llega una línea corrupta
            
        return abs(a * x0 + b * y0 + c) / norm
    
    def predict_point_on_line(self, theta, a, b, c):
        """
        Proyecta dónde debería estar el punto dado el ángulo del rayo láser (theta).
        Es la intersección analítica entre el rayo del LiDAR y la línea extraída.
        """
        # El rayo del láser sale del origen con ángulo theta. 
        # Su vector director es (cos(theta), sin(theta)).
        cos_th = math.cos(theta)
        sin_th = math.sin(theta)
        
        # El denominador es el producto punto entre la normal de la línea y el rayo.
        den = a * cos_th + b * sin_th
        
        # Si el denominador es 0, el rayo láser es paralelo a la pared 
        # (físicamente casi imposible que rebote y llegue al sensor, pero hay que cubrirlo)
        if abs(den) < 1e-6:
            return float('inf'), float('inf')
            
        # Coordenadas de intersección (fórmulas del paper de Gao et al.)
        x_pred = (-c * cos_th) / den
        y_pred = (-c * sin_th) / den
        
        return (x_pred, y_pred)

    def calculate_endpoints(self, raw_start, raw_end, a, b, c):
        """
        Aplica el Punto 8 del profesor (Endpoint Projection).
        Proyecta los puntos extremos ruidosos ortogonalmente sobre la línea ajustada.
        """
        norm_sq = a**2 + b**2
        if norm_sq < 1e-6:
            return raw_start, raw_end

        def project_point(pt):
            x0, y0 = pt
            # Resolviendo el sistema de ecuaciones:
            # 1. ax + by + c = 0
            # 2. bx - ay + (a*y0 - b*x0) = 0
            x_proj = (b**2 * x0 - a * b * y0 - a * c) / norm_sq
            y_proj = (a**2 * y0 - a * b * x0 - b * c) / norm_sq
            return (x_proj, y_proj)

        start_proj = project_point(raw_start)
        end_proj = project_point(raw_end)

        return start_proj, end_proj

    def polar_to_cartesian(self, msg):
        """
        Filtra el ruido del sensor y convierte los rangos polares a 
        coordenadas cartesianas usando vectorización con NumPy.
        
        Input: msg (sensor_msgs/LaserScan)
        Output: numpy.ndarray de forma (N, 2) con puntos [x, y] válidos.
        """
        # Convertimos la tupla/lista de rangos a un arreglo de NumPy
        ranges = np.array(msg.ranges)
        
        # 1. Generamos un arreglo con todos los ángulos simultáneamente
        # np.arange crea [0, 1, 2, ..., N] y lo multiplicamos por el incremento
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        # 2. Máscara Booleana (El reemplazo del 'if' dentro del 'for')
        # Evaluamos todas las condiciones de golpe. Retorna un arreglo de True/False.
        valid_mask = (np.isfinite(ranges) & 
                      (ranges >= msg.range_min) & 
                      (ranges <= msg.range_max))
        
        # 3. Filtramos rangos y ángulos usando la máscara
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # 4. Transformación trigonométrica vectorizada
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        
        # 5. Apilamos las columnas X e Y para obtener una matriz Nx2
        # Resultado: [[x1, y1], [x2, y2], ..., [xN, yN]]
        valid_points = np.column_stack((x, y))
        
        return valid_points

    ### ===== Funciones para evaluacion de casos de solapamiento ===== ###
    def are_lines_collinear(self, a1, b1, a2, b2, angle_threshold_rad=0.1):
        """
        Caso 1 del profesor: Verifica si dos líneas son colineales calculando 
        el ángulo entre sus vectores normales mediante el producto punto.
        """
        # Como los parámetros de Mínimos Cuadrados Ortogonales usualmente 
        # están normalizados (a^2 + b^2 = 1), el producto punto es el coseno del ángulo.
        dot_product = abs(a1 * a2 + b1 * b2)
        
        # Evitamos errores de precisión de punto flotante (> 1.0)
        dot_product = min(1.0, dot_product)
        
        angle = math.acos(dot_product)
        
        # Si el ángulo es menor al umbral (ej. ~5.7 grados), son colineales
        return angle < angle_threshold_rad

    def resolve_corner_conflict(self, point, a1, b1, c1, a2, b2, c2):
        """
        Caso 2 del profesor: Devuelve 1 si el punto está más cerca de la Línea 1, 
        o 2 si está más cerca de la Línea 2.
        """
        d1 = self.distance_point_to_line(point, a1, b1, c1)
        d2 = self.distance_point_to_line(point, a2, b2, c2)
        
        return 1 if d1 <= d2 else 2

    ### ===== Funciones para visualización ===== ###
### ===== Funciones para visualización ===== ###

    def publish_extracted_lines(self, lines):
        """
        Publica los segmentos extraídos como un MarkerArray para visualizarlos en RViz2.
        Input: Lista de objetos LineModel con sus start_point y end_point calculados.
        """
        # Es buena práctica importar los mensajes específicos aquí si no se usan 
        # en todo el archivo, pero si ya los pusiste arriba, no hay problema.
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        
        marker_array = MarkerArray()

        # 1. Marcador de Limpieza (Crucial para robótica móvil)
        # Esto le dice a RViz: "Borra todas las líneas del frame anterior antes de dibujar las nuevas"
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 2. Creación de Marcadores Geométricos
        for i, line_model in enumerate(lines):
            marker = Marker()
            
            # --- Configuración del Header ---
            # ¡IMPORTANTE!: Este frame_id debe coincidir exactamente con el de tu LiDAR.
            # Usualmente es 'laser_frame', 'base_laser' o 'lidar_link'.
            marker.header.frame_id = "lidar_link" 
            marker.header.stamp = self.get_clock().now().to_msg()
            
            # --- Identificación ---
            marker.ns = "extracted_walls"
            marker.id = i # Cada línea debe tener un ID único en este arreglo
            
            # --- Geometría ---
            # LINE_STRIP conecta los puntos que le pasemos en orden
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Escala (El grosor de la línea en metros)
            marker.scale.x = self.line_width
            
            # --- Color (Verde Neón) ---
            # Tip: Podrías hacer que el color dependa del ID para ver cada segmento 
            # de un color distinto y depurar el solapamiento más fácilmente.
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0 # Alpha (1.0 = totalmente opaco)

            # --- Vértices ---
            # Utilizamos los puntos proyectados matemáticamente por calculate_endpoints
            p_start = Point()
            p_start.x = float(line_model.start_point[0])
            p_start.y = float(line_model.start_point[1])
            p_start.z = 0.0 # Z es 0 porque es un LiDAR 2D

            p_end = Point()
            p_end.x = float(line_model.end_point[0])
            p_end.y = float(line_model.end_point[1])
            p_end.z = 0.0

            marker.points.append(p_start)
            marker.points.append(p_end)

            # Agregamos el marcador individual al arreglo global
            marker_array.markers.append(marker)

        # 3. Publicación
        self.lines_pub.publish(marker_array)
    

def main():
    rclpy.init()
    node = LineSegment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

