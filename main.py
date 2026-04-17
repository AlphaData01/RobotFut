
# RobotFutbol CUCEI

import cv2
import time
from Vision import BallTracker
from enum import Enum, auto
import Control

# Camara a usar, Resolucion de la camara
tracker = BallTracker(cam_index=1, width=640, height=480, show_windows=True)

class Estado(Enum):
    BUSQUEDA = auto()
    ALINEAR = auto()
    AVANZAR = auto()
    CAPTURAR = auto()
    TIRAR = auto()

# Tolerancias en pixeles
TOL_X = 60
TOL_Y = 40

t_empuje = None

def main():
    estado = Estado.BUSQUEDA

    # Conectar con Arduino
    Control.connect("COM5")

    # Limitar frecuencia de envío (evita Write timeout)
    last_send = 0.0
    SEND_DT = 0.03  # 33 Hz (si aún falla, prueba 0.05)

    # Tiempo de captura
    t_capturar = None
    CAPTURAR_TIMEOUT = 5.0

    # Tiempo para patear
    t_tirar = None

    # Rodillo de Captura
    cilindro_on = 0

    # Estados
    estado_prev = None

    try:
        while True:
            #Valores de la vision
            x, y, r, found, capture, debug, error_x, error_y = tracker.read()

            # Fallo camara
            if debug is None:
                break

            # Camara
            cv2.imshow("Salida", debug)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Valores del Arduino en cada Iteracion
            Ux = 0
            Uy = 0
            Ut = 0
            patada = 0
            cilindro = cilindro_on

            # ---- MAQUINA DE ESTADOS----

            # ========== BUSQUEDA ==========
            if estado == Estado.BUSQUEDA:
                # Imprimir estado
                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado
                # ---- TRANSICIONES ----
                if found:
                    estado = Estado.ALINEAR
                else:
                    Uy = 4 # Ux Avanzar
                    #Ux = - 0.4 # Girar Buscando
                    pass

            # ========== ALINEAR ==========
            elif estado == Estado.ALINEAR:
                # Imprimir estado
                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado
                # ---- Si perdio la pelota, regresa a BUSQUEDA ----
                if not found:
                    print("Perdí pelota en ALINEAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                Ut = 0.01 * error_x
                # Ut = Control.ut_from_error(error_x)

                # ---- TRANSICIONES ----
                if abs(error_x) <= TOL_X:
                    estado = Estado.AVANZAR

            # ========== AVANZAR ==========
            elif estado == Estado.AVANZAR:
                # Imprimir estado
                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado
                # ---- Si perdio la pelota, regresa a BUSQUEDA ----
                if not found:
                    print("Perdí pelota en AVANZAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                # Error X
                Ut = 0.008 * error_x

                # Error Y
                Ux = 0.0025 * error_y

                # ---- TRANSICIONES ----
                if abs(error_y) <= TOL_Y:
                    estado = Estado.CAPTURAR

            # ========== CAPTURAR ===========
            elif estado == Estado.CAPTURAR:
                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado
                    t_empuje = None  # reset empuje al entrar

                cilindro_on = 1
                Ut = 0

                # Timer general de captura (NO lo tocamos)
                if t_capturar is None:
                    t_capturar = time.time()

                # Timer solo para el empujón
                if t_empuje is None:
                    t_empuje = time.time()

                # 👉 Empuje corto (1.2 segundos)
                if time.time() - t_empuje < 1.2:
                    #Ut = - 0.09
                    Ux = - 0.3
                else:
                    Ux = 0

                # Leer sensor de pelota
                pelota = Control.read()

                # ---- TRANSICIONES ----
                if pelota == 1:
                    print("Pelota detectada -> TIRAR")
                    t_capturar = None
                    t_empuje = None
                    estado = Estado.TIRAR

                elif time.time() - t_capturar > CAPTURAR_TIMEOUT:
                    print("No capturó -> BUSQUEDA")
                    cilindro_on = 0
                    t_capturar = None
                    t_empuje = None
                    estado = Estado.BUSQUEDA

            # ========== TIRAR ===========
            elif estado == Estado.TIRAR:
                # Imprimir estado
                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado

                cilindro_on = 1
                Ux = 0
                Uy = 0
                Ut = 0

                # Si es la primera vez que entra
                if t_tirar is None:
                    t_tirar = time.time()

                # Espera 2 segundos con cilindro girando
                if time.time() - t_tirar >= 2:
                    patada = 1  # Activa pateador
                    cilindro_on = 0  # Apaga cilindro
                    t_tirar = None  # Reset
                    estado = Estado.BUSQUEDA

            # print("ENVIO:", Ux, Uy, Ut, patada, cilindro)
            # ENVIAR A 33 Hz (no cada frame)
            now = time.time()
            if (now - last_send) >= SEND_DT:
                Control.send(Ux, Uy, Ut, patada, cilindro)
                last_send = now

    finally:
        tracker.release()

if __name__ == "__main__":
    main()

