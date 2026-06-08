import cv2
from vision import Vision


vision = Vision(cam_id=0, mostrar=True)

while True:
    datos = vision.leer()

    if datos is None:
        break

    print(
        "X:", datos["x"],
        "Y:", datos["y"],
        "Dentro X:", datos["dentro_x"],
        "Dentro Y:", datos["dentro_y"],
        "Dentro rango:", datos["dentro_rango"]
    )

    if datos["dentro_rango"]:
        print("La pelota está en zona de captura")

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

vision.cerrar()