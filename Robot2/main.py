import cv2

from Vision import Vision
from vision_husky import VisionHusky
import Control


# ---------------- CONEXION SERIAL ----------------
# Ubuntu normalmente:
Control.connect("/dev/ttyACM0")

# Windows seria algo como:
# Control.connect("COM5")


# ---------------- VISION ----------------
vision_360 = Vision(cam_id=0, mostrar=True)
vision_husky = VisionHusky(mostrar=True)


# ---------------- CONTROL 360 ----------------
KP_360_X = 0.01
KP_360_Y = 0.01
V_MAX = 1.0


def limitar(valor):
    return max(min(valor, V_MAX), -V_MAX)


try:
    while True:

        vx = 0
        vy = 0
        estado = "INICIO"

        # =====================================================
        # PRIORIDAD 1: HUSKY
        # =====================================================
        datos_husky = vision_husky.leer()

        if datos_husky["detectada"]:

            vx = datos_husky["vx"]
            vy = datos_husky["vy"]
            estado = datos_husky["estado"]

        # =====================================================
        # PRIORIDAD 2: CAMARA 360
        # =====================================================
        else:

            datos_360 = vision_360.leer()

            if datos_360 is None:
                break

            if datos_360["valida"]:

                if datos_360["dentro_rango"]:
                    vx = 0
                    vy = 0
                    estado = "360 ESPERANDO HUSKY"

                else:
                    error_x = datos_360["x"] - vision_360.CAP_X
                    error_y = datos_360["y"] - vision_360.CAP_Y

                    vx = -KP_360_X * error_x
                    vy = -KP_360_Y * error_y

                    vx = limitar(vx)
                    vy = limitar(vy)

                    estado = "360 ALINEAR"

            else:
                vx = 0
                vy = 0
                estado = "SIN PELOTA"

        # =====================================================
        # ENVIAR AL ROBOT
        # =====================================================
        Control.send(
            Ux=vx,
            Uy=vy,
            Ut=0,
            patada=0,
            cilindro=0,
            modo="G"
        )

        print(f"[{estado}] Ux={vx:.2f} Uy={vy:.2f}")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


except KeyboardInterrupt:
    print("Programa detenido")


finally:
    Control.send(0, 0, 0)
    Control.close()
    vision_360.cerrar()
    cv2.destroyAllWindows()