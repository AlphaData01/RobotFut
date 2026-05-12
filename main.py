# main.py
# RobotFutbol CUCEI - Robot fisico
# Adaptado desde la simulacion, respetando ecuaciones directas:
# Ut = -0.008 * error_x

import cv2
import time
from enum import Enum, auto

import Control
from Vision import BallTracker

# ==========================
# CAMARA
# ==========================
tracker = BallTracker(cam_index=0, width=640, height=480, show_windows=True)

# ==========================
# ESTADOS
# ==========================
class Estado(Enum):
    BUSQUEDA = auto()
    ALINEAR = auto()
    AVANZAR = auto()
    CAPTURAR = auto()
    PORTERIA = auto()
    TIRAR = auto()

# ==========================
# TOLERANCIAS
# ==========================
TOL_X = 30
TOL_Y = 55
TOL_PORTERIA_X = 25

# ==========================
# TIEMPOS
# ==========================
CAPTURAR_TIMEOUT = 1.0
TIEMPO_EMPUJE = 0.5
TIEMPO_PULSO_PATEO_INICIO = 0.1
TIEMPO_PULSO_PATEO_FIN = 0.6
TIEMPO_POST_PATEO = 2.0

# Frecuencia de envio serial
SEND_DT = 0.03  # 33 Hz

# ==========================
# BUSQUEDA LATERAL
# ==========================
TIEMPO_LATERAL_BUSQUEDA = 2.0
TIEMPO_RECENTRAR_BUSQUEDA = 1.0
VEL_LATERAL_BUSQUEDA = 0.25


def main():
    estado = Estado.BUSQUEDA
    estado_prev = None

    t_capturar = None
    t_empuje = None
    t_tirar = None
    t_post_pateo = None

    cilindro_on = 0
    last_send = 0.0

    # BUSQUEDA LATERAL
    t_busqueda_lateral = None
    direccion_lateral = 1
    modo_busqueda_lateral = False

    # Conectar con Arduino
    # Ubuntu: /dev/ttyACM0, /dev/ttyUSB0, etc.
    Control.connect("COM5", 115200)

    try:
        while True:
            # ==========================
            # VISION
            # ==========================
            x, y, r, found, capture, error_x_goal, debug, error_x, error_y = tracker.read()

            if debug is None:
                print("Fallo camara")
                break

            cv2.imshow("Salida", debug)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # ==========================
            # LECTURA ARDUINO
            # ==========================
            pelota = Control.read()

            # ==========================
            # VALORES A ENVIAR
            # ==========================
            Ux = 0
            Uy = 0
            Ut = 0
            patada = 0
            cilindro = cilindro_on

            # ==========================
            # BUSQUEDA
            # ==========================
            if estado == Estado.BUSQUEDA:

                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado
                    t_busqueda_lateral = None
                    modo_busqueda_lateral = False

                cilindro_on = 0
                cilindro = cilindro_on
                patada = 0

                # Si ve pelota, rompe cualquier rutina y va por ella
                if found:
                    print("Pelota encontrada -> ALINEAR")
                    Ux = 0
                    Uy = 0
                    Ut = 0
                    t_busqueda_lateral = None
                    modo_busqueda_lateral = False
                    estado = Estado.ALINEAR

                else:
                    # Si no ve porteria, se mueve hacia atras
                    if error_x_goal is None:
                        Ux = -0.22
                        Uy = 0
                        Ut = 0
                        t_busqueda_lateral = None
                        modo_busqueda_lateral = False

                    # Si ve porteria pero no esta alineado, centra porteria
                    elif abs(error_x_goal) > TOL_PORTERIA_X:
                        Ux = -0.018
                        Uy = -0.004 * error_x_goal
                        Ut = 0
                        t_busqueda_lateral = None
                        modo_busqueda_lateral = False

                    # Si ya esta alineado con porteria, inicia rutina lateral
                    else:
                        if not modo_busqueda_lateral:
                            modo_busqueda_lateral = True
                            t_busqueda_lateral = time.time()
                            direccion_lateral *= -1
                            print("Porteria centrada -> busqueda lateral")

                        tiempo_lateral = time.time() - t_busqueda_lateral

                        # Mover lateralmente unos segundos
                        if tiempo_lateral < TIEMPO_LATERAL_BUSQUEDA:
                            Ux = 0
                            Uy = direccion_lateral * VEL_LATERAL_BUSQUEDA
                            Ut = 0

                        # Termino lateral, vuelve a recentrar porteria
                        else:
                            Ux = 0
                            Uy = 0
                            Ut = 0
                            modo_busqueda_lateral = False
                            t_busqueda_lateral = None
                            print("Termino lateral -> recentrar porteria")
            # ==========================
            # ALINEAR PELOTA
            # ==========================
            elif estado == Estado.ALINEAR:

                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado

                if not found:
                    print("Perdi pelota en ALINEAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue

                Uy = -0.004 * error_x

                if abs(error_x) <= TOL_X:
                    estado = Estado.AVANZAR

            # ==========================
            # AVANZAR HACIA PELOTA
            # ==========================
            elif estado == Estado.AVANZAR:

                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado

                if not found:
                    print("Perdi pelota en AVANZAR -> BUSQUEDA")
                    estado = Estado.BUSQUEDA
                    continue
                
                Ux = 0.35
                Uy = -0.005 * error_x

                if abs(error_y) <= TOL_Y:
                    estado = Estado.CAPTURAR

            # ==========================
            # CAPTURAR PELOTA
            # ==========================
            elif estado == Estado.CAPTURAR:

                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado
                    t_empuje = None

                cilindro_on = 1
                cilindro = cilindro_on

                if t_capturar is None:
                    t_capturar = time.time()

                if t_empuje is None:
                    t_empuje = time.time()

                # Empuje corto para meter pelota al rodillo
                if time.time() - t_empuje < TIEMPO_EMPUJE:
                    Ux = -0.30
                else:
                    Ux = 0

                Ut = 0

                if pelota == 1:
                    print("Pelota capturada -> PORTERIA")
                    t_capturar = None
                    t_empuje = None
                    estado = Estado.PORTERIA

                elif time.time() - t_capturar > CAPTURAR_TIMEOUT:
                    print("No capturo -> BUSQUEDA")
                    cilindro_on = 0
                    cilindro = cilindro_on
                    t_capturar = None
                    t_empuje = None
                    estado = Estado.BUSQUEDA

            # ==========================
            # BUSCAR / ALINEAR PORTERIA
            # ==========================
            elif estado == Estado.PORTERIA:

                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado

                cilindro_on = 1
                cilindro = cilindro_on
                patada = 0

                # 1. Si no ve porteria, gira buscandola y avanza poquito
                if error_x_goal is None:
                    Ux = -0.25
                    Uy = 0
                    Ut = 0

                # 2. Si ve porteria pero no esta alineada
                elif abs(error_x_goal) > TOL_PORTERIA_X:
                    Ux = 0.25
                    Uy = -0.005 * error_x_goal
                    Ut = 0

                # 3. Si esta casi alineada, avanza manteniendo correccion pequena
                else:
                    Ux = 0.25
                    Uy = -0.001 * error_x_goal
                    Ut = 0

                # 4. Disparar solo si ve porteria y esta alineado
                if error_x_goal is not None and abs(error_x_goal) < TOL_PORTERIA_X:
                    print("Porteria alineada -> TIRAR")
                    Ux = 0
                    Uy = 0
                    Ut = 0
                    estado = Estado.TIRAR

            # ==========================
            # TIRAR
            # ==========================
            elif estado == Estado.TIRAR:

                if estado != estado_prev:
                    print("ESTADO:", estado.name)
                    estado_prev = estado
                    t_tirar = time.time()
                    t_post_pateo = None

                Ux = 0
                Uy = 0
                Ut = 0
                cilindro_on = 1
                cilindro = cilindro_on

                tiempo_tiro = time.time() - t_tirar

                # Pulso de pateo, como en simulacion
                if TIEMPO_PULSO_PATEO_INICIO < tiempo_tiro < TIEMPO_PULSO_PATEO_FIN:
                    patada = 1
                else:
                    patada = 0

                # Iniciar espera post-pateo
                if tiempo_tiro >= TIEMPO_PULSO_PATEO_FIN and t_post_pateo is None:
                    t_post_pateo = time.time()

                # Esperar despues de tirar
                if t_post_pateo is not None:
                    if time.time() - t_post_pateo >= TIEMPO_POST_PATEO:
                        cilindro_on = 0
                        cilindro = cilindro_on
                        t_tirar = None
                        t_post_pateo = None
                        estado = Estado.BUSQUEDA

            # ==========================
            # ENVIAR A ARDUINO
            # ==========================
            now = time.time()
            if (now - last_send) >= SEND_DT:
                Control.send(Ux, Uy, Ut, patada, cilindro, "G")
                last_send = now

    # Ux, Uy, Ut, patada, cilindro, modo="G"
    finally:
        # Detener robot al cerrar
        try:
            Control.send(Ux, Uy, Ut, patada, cilindro, modo="G")
        except Exception:
            pass

        tracker.release()
        Control.close()


if __name__ == "__main__":
    main()
