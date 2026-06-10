import cv2

from Vision import Vision
from vision_husky import VisionHusky
import Control


# ---------------- CONEXION SERIAL ----------------
# Ubuntu normalmente:
Control.connect("/dev/ttyUSB0")

# Windows seria algo como:
# Control.connect("COM5")


# ---------------- VISION ----------------
vision_360 = Vision(cam_id=0, mostrar=True)
vision_husky = VisionHusky(mostrar=True)


# ---------------- CONTROL 360 ----------------
KP_360_X = 0.005
KP_360_Y = 0.004
V_MAX_360 = 0.5

# ---------------- CONTROL HUSKY ----------------
V_MAX_HUSKY = 0.3


def limitar(valor, v_max):
    return max(min(valor, v_max), -v_max)


try:
    while True:

        vx = 0
        vy = 0
        estado = "INICIO"
        fuente = "NINGUNA"

        # =====================================================
        # PRIORIDAD 1: HUSKY
        # =====================================================
        datos_husky = vision_husky.leer()

        if datos_husky["detectada"]:

            # IMPORTANTE:
            # Se respetan tus ejes cruzados:
            # lo que Husky calcula como vx se manda a vy
            # lo que Husky calcula como vy se manda a vx
            vy = datos_husky["vx"]
            vx = datos_husky["vy"]

            vx = limitar(vx, V_MAX_HUSKY)
            vy = limitar(vy, V_MAX_HUSKY)

            estado = datos_husky["estado"]
            fuente = "HUSKY"

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
                    fuente = "360"

                else:
                    error_x = datos_360["x"] - vision_360.CAP_X
                    error_y = datos_360["y"] - vision_360.CAP_Y

                    # Se respetan tus signos originales
                    vx = KP_360_X * error_x
                    vy = KP_360_Y * error_y

                    vx = limitar(vx, V_MAX_360)
                    vy = limitar(vy, V_MAX_360)

                    estado = "360 ALINEAR"
                    fuente = "360"

            else:
                vx = 0
                vy = 0
                estado = "SIN PELOTA"
                fuente = "NINGUNA"

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

        print(
            f"[{fuente} | {estado}] "
            f"Ux={vx:.2f} Uy={vy:.2f}"
        )

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


except KeyboardInterrupt:
    print("Programa detenido")


finally:
    Control.send(0, 0, 0)
    Control.close()
    vision_360.cerrar()
    cv2.destroyAllWindows()