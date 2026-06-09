import cv2
import numpy as np

from Vision import Vision
from vision_husky import VisionHusky


# ---------------- CAMARAS ----------------
vision_360 = Vision(cam_id=0, mostrar=True)
vision_husky = VisionHusky()


# ---------------- AJUSTES HUSKY ----------------
HUSKY_W = 320
HUSKY_H = 240

# Punto/zona donde quieres capturar con Husky
HUSKY_CAP_X = 160
HUSKY_CAP_Y = 190

HUSKY_TOL_X = 35
HUSKY_TOL_Y = 20


# ---------------- CONTROL ----------------
KP_360_X = 0.01
KP_360_Y = 0.01

KP_HUSKY_X = 0.01
KP_HUSKY_Y = 0.01

V_MAX = 1.0


def limitar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo)


def dibujar_husky(pelota, vx, vy, estado):
    # Fondo negro
    frame = np.zeros((HUSKY_H, HUSKY_W, 3), dtype=np.uint8)

    # Lineas rojas verticales
    x1 = HUSKY_CAP_X - HUSKY_TOL_X
    x2 = HUSKY_CAP_X + HUSKY_TOL_X

    cv2.line(frame, (x1, 0), (x1, HUSKY_H), (0, 0, 255), 3)
    cv2.line(frame, (x2, 0), (x2, HUSKY_H), (0, 0, 255), 3)

    # Linea verde horizontal de captura
    cv2.line(frame, (0, HUSKY_CAP_Y), (HUSKY_W, HUSKY_CAP_Y), (0, 255, 0), 3)

    # Punto central de captura
    cv2.circle(frame, (HUSKY_CAP_X, HUSKY_CAP_Y), 5, (0, 255, 255), -1)

    if pelota is not None:
        x = int(pelota["x"])
        y = int(pelota["y"])
        w = int(pelota["w"])
        h = int(pelota["h"])

        # Dibujar pelota
        cv2.circle(frame, (x, y), 8, (0, 165, 255), -1)
        cv2.rectangle(
            frame,
            (x - w // 2, y - h // 2),
            (x + w // 2, y + h // 2),
            (0, 255, 255),
            2
        )

        cv2.putText(frame, "Pelota ID1", (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.putText(frame, estado, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.putText(frame, f"VX={vx:.2f} VY={vy:.2f}", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("Husky", frame)


while True:

    vx = 0
    vy = 0

    # ---------------- LEER HUSKY ----------------
    pelota_husky = vision_husky.get_pelota()

    # ============================================================
    # PRIORIDAD 1: HUSKY VE LA PELOTA
    # ============================================================
    if pelota_husky is not None:

        x = pelota_husky["x"]
        y = pelota_husky["y"]

        error_x = x - HUSKY_CAP_X
        error_y = y - HUSKY_CAP_Y

        dentro_x = abs(error_x) <= HUSKY_TOL_X
        dentro_y = y >= HUSKY_CAP_Y - HUSKY_TOL_Y

        dentro_captura = dentro_x and dentro_y

        if dentro_captura:
            vx = 0
            vy = 0
            estado = "HUSKY CAPTURAR"

        else:
            vx = -KP_HUSKY_X * error_x
            vy = -KP_HUSKY_Y * error_y

            vx = limitar(vx, -V_MAX, V_MAX)
            vy = limitar(vy, -V_MAX, V_MAX)

            estado = "HUSKY ALINEAR"

        print(
            f"[{estado}] "
            f"x={x} y={y} "
            f"error_x={error_x:.0f} error_y={error_y:.0f} "
            f"vx={vx:.2f} vy={vy:.2f}"
        )

        dibujar_husky(pelota_husky, vx, vy, estado)

    # ============================================================
    # PRIORIDAD 2: SI HUSKY NO VE, USA CAMARA 360
    # ============================================================
    else:

        datos_360 = vision_360.leer()

        if datos_360 is None:
            break

        if datos_360["valida"]:

            if datos_360["dentro_rango"]:
                vx = 0
                vy = 0
                estado = "360 CAPTURAR / ESPERAR HUSKY"

            else:
                error_x = datos_360["x"] - vision_360.CAP_X
                error_y = datos_360["y"] - vision_360.CAP_Y

                vx = -KP_360_X * error_x
                vy = -KP_360_Y * error_y

                vx = limitar(vx, -V_MAX, V_MAX)
                vy = limitar(vy, -V_MAX, V_MAX)

                estado = "360 ALINEAR"

            print(
                f"[{estado}] "
                f"x={datos_360['x']} y={datos_360['y']} "
                f"vx={vx:.2f} vy={vy:.2f}"
            )

        else:
            vx = 0
            vy = 0
            estado = "SIN PELOTA"
            print(f"[{estado}] vx={vx:.2f} vy={vy:.2f}")

        dibujar_husky(None, vx, vy, "HUSKY SIN PELOTA")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break


vision_360.cerrar()
cv2.destroyAllWindows()