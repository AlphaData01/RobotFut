import cv2
from Vision import Vision

# Dar valores a la clase
vision = Vision(cam_id=0, mostrar=True)

# Ganancias
KP_X = 0.01
KP_Y = 0.01

# Velocidad
V_MAX = 1.0

while True:

    # Funcion principal de vision
    datos = vision.leer()

    if datos is None:
        break

    if datos["valida"]:

        if datos["dentro_rango"]:

            vx = 0
            vy = 0

            print(
                f"CAPTURAR | VX={vx:.2f} VY={vy:.2f}"
            )

        else:

            # Error respecto al punto de captura
            error_x = datos["x"] - vision.CAP_X
            error_y = datos["y"] - vision.CAP_Y

            # Control proporcional
            vx = -KP_X * error_x
            vy = -KP_Y * error_y

            # Saturacion
            vx = max(min(vx, V_MAX), -V_MAX)
            vy = max(min(vy, V_MAX), -V_MAX)

            print(f"VX={vx:5.2f} "
                  f"VY={vy:5.2f}"
                  )

    else:

        print("SIN PELOTA")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

vision.cerrar()