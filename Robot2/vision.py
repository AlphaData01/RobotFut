import cv2
import numpy as np

cam = cv2.VideoCapture(0)

# Color naranja de la pelota
bajo = np.array([7, 120, 120])
alto = np.array([12, 255, 255])

# ---------- AJUSTES ----------
CX, CY = 320, 240
RADIO_ROBOT = 170

CAP_X, CAP_Y = 490, 240

TOL_X = 55
TOL_Y = 35

# ---------- KALMAN ----------
kalman = cv2.KalmanFilter(4, 2)

kalman.transitionMatrix = np.array([[1,0,1,0],
                                    [0,1,0,1],
                                    [0,0,1,0],
                                    [0,0,0,1]], np.float32)

kalman.measurementMatrix = np.array([[1,0,0,0],
                                     [0,1,0,0]], np.float32)

kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5

iniciado = False

while True:
    ret, frame = cam.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    # Centro real de la imagen
    centro_img = (w // 2, h // 2)

    # ---------- DETECCIÓN DE PELOTA ----------
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, bajo, alto)

    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    pred = kalman.predict()
    px, py = int(pred[0, 0]), int(pred[1, 0])

    if contornos:
        c = max(contornos, key=cv2.contourArea)

        if cv2.contourArea(c) >= 1:
            (x, y), r = cv2.minEnclosingCircle(c)
            x, y, r = int(x), int(y), max(int(r), 8)

            if not iniciado:
                kalman.statePost = np.array([[x], [y], [0], [0]], np.float32)
                iniciado = True

            kalman.correct(np.array([[x], [y]], np.float32))

            # Pelota real detectada
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
            cv2.putText(frame, "Pelota", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # ---------- KALMAN ----------
    if iniciado:
        # Punto azul = predicción Kalman
        cv2.circle(frame, (px, py), 5, (255, 0, 0), -1)
        cv2.putText(frame, "Kalman", (px + 8, py + 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

    # ---------- DIBUJOS DEL ROBOT ----------

    # Círculo negro del robot
    cv2.circle(frame, (CX, CY), RADIO_ROBOT, (0, 0, 0), 3)

    # Punto naranja de captura
    cv2.circle(frame, (CAP_X, CAP_Y), 7, (0, 100, 255), -1)

    # Punto rojo en el mero centro de la imagen
    cv2.circle(frame, centro_img, 5, (0, 0, 255), -1)

    # Línea azul izquierda empieza justo donde está el punto naranja
    cv2.line(frame, (CAP_X, 0), (CAP_X, h), (255, 0, 0), 3)

    # Línea azul derecha es la tolerancia
    cv2.line(frame, (CAP_X + TOL_X, 0), (CAP_X + TOL_X, h), (255, 0, 0), 3)

    # Líneas rosas horizontales de tolerancia
    cv2.line(frame, (0, CAP_Y - TOL_Y), (w, CAP_Y - TOL_Y), (255, 0, 255), 3)
    cv2.line(frame, (0, CAP_Y + TOL_Y), (w, CAP_Y + TOL_Y), (255, 0, 255), 3)

    cv2.imshow("camara", frame)
    cv2.imshow("naranja", mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cam.release()
cv2.destroyAllWindows()