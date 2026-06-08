import cv2
import numpy as np


class Vision:
    def __init__(self, cam_id=0, mostrar=True):
        self.cam = cv2.VideoCapture(cam_id)
        self.mostrar = mostrar

        # Color naranja
        self.bajo = np.array([7, 120, 120])
        self.alto = np.array([12, 255, 255])

        # Ajustes de captura
        self.CX, self.CY = 320, 240
        self.RADIO_ROBOT = 170

        self.CAP_X, self.CAP_Y = 490, 240

        self.TOL_X = 55
        self.TOL_Y = 35

        # Kalman
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

    def leer(self):
        ret, frame = self.cam.read()

        if not ret:
            return None

        h, w = frame.shape[:2]
        centro_img = (w // 2, h // 2)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.bajo, self.alto)

        contornos, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        pred = self.kalman.predict()
        px, py = int(pred[0, 0]), int(pred[1, 0])

        pelota_detectada = False
        x, y, r = None, None, None

        if contornos:
            c = max(contornos, key=cv2.contourArea)

            if cv2.contourArea(c) >= 1:
                (x, y), r = cv2.minEnclosingCircle(c)
                x, y, r = int(x), int(y), max(int(r), 8)

                if not self.iniciado:
                    self.kalman.statePost = np.array([[x], [y], [0], [0]], np.float32)
                    self.iniciado = True

                self.kalman.correct(np.array([[x], [y]], np.float32))
                pelota_detectada = True

        # Si ya inició Kalman, usamos la predicción
        if self.iniciado:
            pelota_x = px
            pelota_y = py
        else:
            pelota_x = x
            pelota_y = y

        dentro_x = False
        dentro_y = False
        dentro_rango = False

        if pelota_x is not None and pelota_y is not None:
            dentro_x = self.CAP_X <= pelota_x <= self.CAP_X + self.TOL_X
            dentro_y = self.CAP_Y - self.TOL_Y <= pelota_y <= self.CAP_Y + self.TOL_Y
            dentro_rango = dentro_x and dentro_y

        if self.mostrar:
            self._dibujar(
                frame, mask, centro_img, pelota_detectada,
                x, y, r, px, py, dentro_rango
            )

        return {
            "detectada": pelota_detectada,
            "x": pelota_x,
            "y": pelota_y,
            "dentro_x": dentro_x,
            "dentro_y": dentro_y,
            "dentro_rango": dentro_rango
        }

    def _dibujar(self, frame, mask, centro_img, pelota_detectada, x, y, r, px, py, dentro_rango):
        h, w = frame.shape[:2]

        if pelota_detectada:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
            cv2.putText(frame, "Pelota", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self.iniciado:
            cv2.circle(frame, (px, py), 5, (255, 0, 0), -1)
            cv2.putText(frame, "Kalman", (px + 8, py + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.circle(frame, (self.CX, self.CY), self.RADIO_ROBOT, (0, 0, 0), 3)
        cv2.circle(frame, (self.CAP_X, self.CAP_Y), 7, (0, 100, 255), -1)
        cv2.circle(frame, centro_img, 5, (0, 0, 255), -1)

        cv2.line(frame, (self.CAP_X, 0), (self.CAP_X, h), (255, 0, 0), 3)
        cv2.line(frame, (self.CAP_X + self.TOL_X, 0), (self.CAP_X + self.TOL_X, h), (255, 0, 0), 3)

        cv2.line(frame, (0, self.CAP_Y - self.TOL_Y), (w, self.CAP_Y - self.TOL_Y), (255, 0, 255), 3)
        cv2.line(frame, (0, self.CAP_Y + self.TOL_Y), (w, self.CAP_Y + self.TOL_Y), (255, 0, 255), 3)

        texto = "DENTRO" if dentro_rango else "FUERA"
        color = (0, 255, 0) if dentro_rango else (0, 0, 255)

        cv2.putText(frame, texto, (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

        cv2.imshow("camara", frame)
        cv2.imshow("naranja", mask)

    def cerrar(self):
        self.cam.release()
        cv2.destroyAllWindows()