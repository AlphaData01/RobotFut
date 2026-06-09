import cv2
import numpy as np

class Vision:
    # Constructor
    def __init__(self, cam_id=0, mostrar=True):
        self.cam = cv2.VideoCapture(cam_id)
        self.mostrar = mostrar

        # Color Pelota Naranja
        self.naranja_bajo = np.array([0, 100, 100])
        self.naranja_alto = np.array([12, 255, 255])

        # Color Porteria Azul
        
        # Color Porteria Amarilla

        # Ajustes de captura
        self.CX, self.CY = 320, 240 # Centro del robot (punto rojo)
        self.RADIO_ROBOT = 155 # Radio del robot (circulo negro)
        self.CAP_X, self.CAP_Y = 485, 240 # Captura (punto naranja)

        # Separacion entre Lineas
        self.TOL_X = 20
        self.TOL_Y = 35

        # ----- Zona curva de captura -----
        dx_cap = self.CAP_X - self.CX
        dy_cap = self.CAP_Y - self.CY

        self.RADIO_CAPTURA = int(np.sqrt(dx_cap ** 2 + dy_cap ** 2))
        self.ANGULO_CAPTURA = np.degrees(np.arctan2(dy_cap, dx_cap))

        self.TOL_RADIO = self.TOL_X
        self.TOL_ANGULO = 12

        # Memoria de la ultima pelota detectada
        self.ultima_x = None
        self.ultima_y = None
        self.ultimo_r = 3

        # Tolerancia cuando se pierde la pelota
        self.frames_perdidos = 0
        self.MAX_FRAMES_BUSQUEDA = 40
        self.MAX_FRAMES_PERDIDOS = 120

        # Radio donde se busca la pelota si se pierde
        self.RADIO_BUSQUEDA_LOCAL = 80

        # ------ Filtro de Kalman ------
        self.kalman = cv2.KalmanFilter(4, 2)

        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], np.float32)

        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], np.float32)

        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5

        self.iniciado = False

    # Funcion para calcular diferencia de angulos
    def diferencia_angulo(self, a, b):
        return (a - b + 180) % 360 - 180
    
    # Funcion Princial
    def leer(self):
        ret, frame = self.cam.read()

        if not ret:
            return None
        
        # Obtener tamaño
        h, w = frame.shape[:2]
        centro_img = (w // 2, h // 2)

        # Convertir a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Segmentacion por color
        mask = cv2.inRange(hsv, self.naranja_bajo, self.naranja_alto)

        # Agrandar pixeles detectados sin borrar pelota lejana
        kernel = np.ones((2, 2), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        
        # Buscar pixeles blancos
        contornos, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Prediccion de Kalman
        pred = self.kalman.predict()
        px, py = int(pred[0, 0]), int(pred[1, 0])

        pelota_detectada = False
        x, y, r = None, None, None
        estado_pelota = "SIN_PELOTA"
        
        # Si existe algun objeto naranja
        if contornos:

            mejor_contorno = None
            mejor_score = 0

            # Revisar todos los objetos naranjas encontrados
            for c in contornos:
                area = cv2.contourArea(c)

                # Permitir objetos pequeños
                if area < 1:
                    continue

                # Obtener centro aproximado del objeto naranja
                (tx, ty), tr = cv2.minEnclosingCircle(c)
                tx = int(tx)
                ty = int(ty)

                # Ignorar el punto naranja de captura
                distancia_cap = np.sqrt(
                    (tx - self.CAP_X) ** 2 +
                    (ty - self.CAP_Y) ** 2
                )

                if distancia_cap < 18:
                    continue

                # Ignorar objetos muy fuera del espejo
                distancia_centro = np.sqrt(
                    (tx - self.CX) ** 2 +
                    (ty - self.CY) ** 2
                )

                if distancia_centro > self.RADIO_ROBOT + 90:
                    continue

                score = area

                # Si ya tengo una ultima posicion, darle prioridad a lo que aparezca cerca
                if self.ultima_x is not None and self.ultima_y is not None:
                    distancia_ultima = np.sqrt(
                        (tx - self.ultima_x) ** 2 +
                        (ty - self.ultima_y) ** 2
                    )

                    if distancia_ultima <= self.RADIO_BUSQUEDA_LOCAL:
                        score = area + 1000
                    elif self.frames_perdidos > 0:
                        continue

                # Elegir el mejor objeto naranja
                if score > mejor_score:
                    mejor_score = score
                    mejor_contorno = c

            if mejor_contorno is not None:
                # Dibujar el circulo de la pelota
                (x, y), r = cv2.minEnclosingCircle(mejor_contorno)
                x, y, r = int(x), int(y), max(int(r), 3)
                
                # Inicializar Kalman
                if not self.iniciado:
                    self.kalman.statePost = np.array([[x], [y], [0], [0]], np.float32)
                    self.iniciado = True

                # Corregir Kalman con posicion real
                self.kalman.correct(np.array([[x], [y]], np.float32))

                # Guardar ultima pelota detectada
                self.ultima_x = x
                self.ultima_y = y
                self.ultimo_r = r
                self.frames_perdidos = 0

                pelota_detectada = True
                estado_pelota = "REAL"

        # Si no se detecto pelota, aumentar contador
        if not pelota_detectada:
            self.frames_perdidos += 1

        # Usar pelota real, ultima posicion o decir que no hay pelota
        if pelota_detectada:
            pelota_x = x
            pelota_y = y
            pelota_valida = True

        elif self.ultima_x is not None and self.frames_perdidos <= self.MAX_FRAMES_BUSQUEDA:
            pelota_x = self.ultima_x
            pelota_y = self.ultima_y

            x = self.ultima_x
            y = self.ultima_y
            r = self.ultimo_r

            pelota_valida = True
            estado_pelota = "BUSCANDO"

        elif self.ultima_x is not None and self.frames_perdidos <= self.MAX_FRAMES_PERDIDOS:
            pelota_x = self.ultima_x
            pelota_y = self.ultima_y

            x = self.ultima_x
            y = self.ultima_y
            r = self.ultimo_r

            pelota_valida = True
            estado_pelota = "MEMORIA"

        else:
            pelota_x = None
            pelota_y = None
            pelota_valida = False
            estado_pelota = "SIN_PELOTA"

            self.ultima_x = None
            self.ultima_y = None
            self.ultimo_r = 3
            self.iniciado = False

        # Inicializar tolerancias curvas
        dentro_angulo = False
        dentro_radio = False
        dentro_rango = False

        angulo_pelota = None
        radio_pelota = None

        if pelota_x is not None and pelota_y is not None and pelota_valida:
            # Calcular distancia y angulo de la pelota respecto al centro del espejo
            dx = pelota_x - self.CX
            dy = pelota_y - self.CY

            radio_pelota = np.sqrt(dx ** 2 + dy ** 2)
            angulo_pelota = np.degrees(np.arctan2(dy, dx))

            error_angulo = self.diferencia_angulo(angulo_pelota, self.ANGULO_CAPTURA)
            error_radio = radio_pelota - self.RADIO_CAPTURA

            # Verificar rango curvo
            dentro_angulo = abs(error_angulo) <= self.TOL_ANGULO
            dentro_radio = abs(error_radio) <= self.TOL_RADIO

            # Verificar la captura
            dentro_rango = dentro_angulo and dentro_radio

        if self.mostrar:
            self._dibujar(
                frame, mask, centro_img,
                pelota_valida, x, y, r,
                px, py, dentro_rango, estado_pelota
            )

        return {
            "detectada": pelota_detectada,
            "valida": pelota_valida,
            "estado_pelota": estado_pelota,
            "x": pelota_x,
            "y": pelota_y,
            "kalman_x": px,
            "kalman_y": py,
            "radio": radio_pelota,
            "angulo": angulo_pelota,
            "frames_perdidos": self.frames_perdidos,
            "dentro_angulo": dentro_angulo,
            "dentro_radio": dentro_radio,
            "dentro_rango": dentro_rango
        }
    
    # ----- Dibujar zona curva -----
    def dibujar_zona_curva(self, frame):
        # Arcos de radio interno y externo
        radio_interno = int(self.RADIO_CAPTURA - self.TOL_RADIO)
        radio_externo = int(self.RADIO_CAPTURA + self.TOL_RADIO)

        angulo_inicio = int(self.ANGULO_CAPTURA - self.TOL_ANGULO)
        angulo_fin = int(self.ANGULO_CAPTURA + self.TOL_ANGULO)

        # OpenCV usa grados desde el eje X
        cv2.ellipse(
            frame,
            (self.CX, self.CY),
            (radio_interno, radio_interno),
            0,
            angulo_inicio,
            angulo_fin,
            (255, 0, 0),
            3
        )

        cv2.ellipse(
            frame,
            (self.CX, self.CY),
            (radio_externo, radio_externo),
            0,
            angulo_inicio,
            angulo_fin,
            (255, 0, 0),
            3
        )

        # Lineas radiales que cierran el sector
        for ang in [angulo_inicio, angulo_fin]:
            rad = np.radians(ang)

            x1 = int(self.CX + radio_interno * np.cos(rad))
            y1 = int(self.CY + radio_interno * np.sin(rad))

            x2 = int(self.CX + radio_externo * np.cos(rad))
            y2 = int(self.CY + radio_externo * np.sin(rad))

            cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 3)

    # ----- Dibujar sobre la pantalla -----
    def _dibujar(self, frame, mask, centro_img, pelota_valida, x, y, r, px, py, dentro_rango, estado_pelota):
        h, w = frame.shape[:2]

        # Punto verde = pelota real o ultima pelota confiable
        if pelota_valida and x is not None and y is not None:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
            cv2.putText(frame, "Pelota", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Punto azul = Kalman solo como referencia visual
        if self.iniciado:
            cv2.circle(frame, (px, py), 5, (255, 0, 0), -1)
            cv2.putText(frame, "K", (px + 8, py + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Circulo limite del robot
        cv2.circle(frame, (self.CX, self.CY), self.RADIO_ROBOT, (0, 0, 0), 3)

        # Punto de captura
        cv2.circle(frame, (self.CAP_X, self.CAP_Y), 7, (0, 100, 255), -1)

        # Punto centro
        cv2.circle(frame, centro_img, 5, (0, 0, 255), -1)

        # Dibujar zona curva de captura
        self.dibujar_zona_curva(frame)

        # Texto principal
        if estado_pelota == "REAL":
            texto = "PELOTA"
            color = (0, 255, 0)
        elif estado_pelota == "BUSCANDO":
            texto = "BUSCANDO"
            color = (0, 255, 255)
        elif estado_pelota == "MEMORIA":
            texto = "MEMORIA"
            color = (0, 165, 255)
        else:
            texto = "SIN PELOTA"
            color = (0, 0, 255)

        cv2.putText(frame, texto, (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

        # Texto secundario
        texto_captura = "CAPTURAR" if dentro_rango else "ALINEAR"
        cv2.putText(frame, texto_captura, (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Dibujar zona de busqueda local
        if self.ultima_x is not None and self.ultima_y is not None and estado_pelota in ["BUSCANDO", "MEMORIA"]:
            cv2.circle(frame, (self.ultima_x, self.ultima_y),
                       self.RADIO_BUSQUEDA_LOCAL, (0, 255, 255), 2)

        # Mostrar ventanas
        cv2.imshow("camara", frame)
        cv2.imshow("Mascara", mask)
    
    # ----- Cerrar ventanas -----
    def cerrar(self):
        self.cam.release()
        cv2.destroyAllWindows()