import time
import serial

ser = None


def connect(port="/dev/ttyUSB0", baud=115200):
    global ser
    ser = serial.Serial(port, baud, timeout=0.01, write_timeout=0.01)
    time.sleep(2)
    print(f"Conectado a Arduino en {port}")


def close():
    global ser
    if ser:
        send(0, 0, 0)
        ser.close()
        print("Serial cerrado")
    ser = None


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def send(Ux=0, Uy=0, Ut=0, patada=0, cilindro=0, modo="G"):
    global ser

    if not ser:
        return

    try:
        Ux = clamp(Ux)
        Uy = clamp(Uy)
        Ut = clamp(Ut)

        line = f"M,{Ux:.3f},{Uy:.3f},{Ut:.3f},{int(patada)},{int(cilindro)},{modo}\n"
        ser.write(line.encode("ascii"))

    except Exception as e:
        print("SERIAL SEND ERROR:", e)
        close()