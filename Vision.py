# Vision.py
# Vision para robot futbol fisico: pelota naranja + porteria azul

import cv2
import numpy as np

# ==========================
# CONFIG HSV
# ==========================
# Pelota naranja
NARANJA_BAJO = np.array([5, 120, 120])
NARANJA_ALTO = np.array([25, 255, 255])

# Porteria azul
AZUL_BAJO = np.array([100, 120, 70])
AZUL_ALTO = np.array([140, 255, 255])

KERNEL = np.ones((5, 5), np.uint8)
KERNEL_GOAL = np.ones((7, 7), np.uint8)

# ==========================
# TRACKING PELOTA
# ==========================
CIRC_MIN = 0.35
AREA_MIN_BASE = 250
MIN_ORANGE_FILL = 0.20

ROI_PAD = 160
MAX_LOST = 12
MAX_JUMP = 350

# ==========================
# GUIAS
# ==========================
GUIDE_CENTER_X = 0.50
GUIDE_TOL_X = 0.05
Y_TOL = 15
CAPTURE_ALPHA = 0.35

# ==========================
# PORTERIA
# ==========================
AREA_PORTERIA_MIN = 800


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def orange_fill_ratio(mask, cx, cy, r):
    h, w = mask.shape

    x1 = clamp(cx - r, 0, w - 1)
    y1 = clamp(cy - r, 0, h - 1)
    x2 = clamp(cx + r, 0, w - 1)
    y2 = clamp(cy + r, 0, h - 1)

    roi = mask[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0

    yy, xx = np.ogrid[:roi.shape[0], :roi.shape[1]]
    cy2 = cy - y1
    cx2 = cx - x1
    circle = (xx - cx2) ** 2 + (yy - cy2) ** 2 <= r * r

    inside = roi[circle]
    if inside.size == 0:
        return 0.0

    return float(np.mean(inside > 0))


def detectar_porteria(frame):
    """
    Detecta la porteria azul en el frame completo.
    Regresa:
    goal_x, goal_y, goal_area
    o None, None, None si no encuentra.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, AZUL_BAJO, AZUL_ALTO)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_GOAL, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL_GOAL, iterations=1)

    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contornos:
        return None, None, None

    c = max(contornos, key=cv2.contourArea)
    area = cv2.contourArea(c)

    if area < AREA_PORTERIA_MIN:
        return None, None, None

    x, y, w, h = cv2.boundingRect(c)

    cx = x + w // 2
    cy = y + h // 2

    return cx, cy, area


class BallTracker:

    def __init__(self, cam_index=0, width=None, height=None, show_windows=False):
        self.cap = cv2.VideoCapture(cam_index)

        if not self.cap.isOpened():
            raise RuntimeError("No se pudo abrir la camara.")

        if width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))

        if height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

        self.show_windows = show_windows
        self.last_center = None
        self.last_radius = None
        self.lost_frames = 0

    def read(self):
        """
        Lee la camara y regresa:
        x, y, r, found, capture, error_x_goal, debug, error_x, error_y
        """
        ret, frame = self.cap.read()

        if not ret:
            return None, None, None, False, False, None, None, None, None

        H, W = frame.shape[:2]

        # ==========================
        # DETECTAR PORTERIA
        # ==========================
        goal_x, goal_y, goal_area = detectar_porteria(frame)

        origin_x = int(W * GUIDE_CENTER_X)
        origin_y = H - 1
        y_tol = H - Y_TOL

        error_x_goal = None
        if goal_x is not None:
            error_x_goal = goal_x - origin_x

        # ==========================
        # ROI PELOTA
        # ==========================
        if self.last_center is not None and self.lost_frames < MAX_LOST:
            cx, cy = self.last_center
            x1 = clamp(cx - ROI_PAD, 0, W - 1)
            y1 = clamp(cy - ROI_PAD, 0, H - 1)
            x2 = clamp(cx + ROI_PAD, 0, W - 1)
            y2 = clamp(cy + ROI_PAD, 0, H - 1)
            roi = frame[y1:y2, x1:x2]
            offx, offy = x1, y1
        else:
            roi = frame
            offx, offy = 0, 0

        # ==========================
        # HSV PELOTA
        # ==========================
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, NARANJA_BAJO, NARANJA_ALTO)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if self.last_radius is not None:
            area_min = max(AREA_MIN_BASE, int(np.pi * (0.6 * self.last_radius) ** 2))
        else:
            area_min = AREA_MIN_BASE

        best_area = -1
        best_circle = None

        for c in contornos:
            area = cv2.contourArea(c)

            if area < area_min:
                continue

            per = cv2.arcLength(c, True)

            if per <= 0:
                continue

            circ = 4 * np.pi * area / (per * per)

            if circ < CIRC_MIN:
                continue

            (x, y), r = cv2.minEnclosingCircle(c)
            x = int(x + offx)
            y = int(y + offy)
            r = int(r)

            if self.last_center is not None and self.lost_frames < MAX_LOST:
                dx = x - self.last_center[0]
                dy = y - self.last_center[1]
                dist = (dx * dx + dy * dy) ** 0.5

                if dist > MAX_JUMP:
                    continue
            else:
                dist = 0.0

            x_roi = x - offx
            y_roi = y - offy
            fill = orange_fill_ratio(mask, x_roi, y_roi, max(5, r - 2))

            if fill < MIN_ORANGE_FILL:
                continue

            if area > best_area:
                best_area = area
                best_circle = (x, y, r, fill, circ, dist, area)

        # ==========================
        # DEBUG
        # ==========================
        debug = frame.copy()

        cxg = origin_x
        tol = int(W * GUIDE_TOL_X)
        xL = cxg - tol
        xR = cxg + tol

        # Zona de captura abajo
        overlay = debug.copy()
        cv2.rectangle(overlay, (0, y_tol), (W - 1, H - 1), (255, 0, 0), -1)
        debug = cv2.addWeighted(overlay, CAPTURE_ALPHA, debug, 1 - CAPTURE_ALPHA, 0)

        # Lineas guia
        cv2.line(debug, (cxg, 0), (cxg, H - 1), (0, 0, 0), 2)
        cv2.line(debug, (xL, 0), (xL, H - 1), (0, 0, 255), 2)
        cv2.line(debug, (xR, 0), (xR, H - 1), (0, 0, 255), 2)
        cv2.line(debug, (0, y_tol), (W - 1, y_tol), (0, 0, 255), 2)

        cv2.circle(debug, (origin_x, origin_y), 4, (0, 0, 0), -1)
        cv2.putText(debug, "(0,0)", (origin_x + 6, origin_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Dibujar porteria
        if goal_x is not None:
            cv2.rectangle(debug, (goal_x - 30, goal_y - 30), (goal_x + 30, goal_y + 30), (255, 0, 0), 2)
            cv2.putText(debug, f"goal_error_x = {error_x_goal:+d}", (10, 165),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)

        # ==========================
        # SI ENCONTRO PELOTA
        # ==========================
        if best_circle is not None:
            x, y, r, fill, circ, dist, area = best_circle

            self.last_center = (x, y)
            self.last_radius = r
            self.lost_frames = 0

            capture = y >= y_tol

            error_x = x - origin_x
            error_y = y - origin_y

            cv2.circle(debug, (x, y), r, (0, 255, 0), 2)
            cv2.circle(debug, (x, y), 3, (0, 0, 255), -1)

            cv2.putText(debug, f"error_x = {error_x:+d}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
            cv2.putText(debug, f"error_y = {error_y:+d}", (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 0), 2)
            cv2.putText(debug, f"r={r} fill={fill:.2f} circ={circ:.2f}", (10, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if capture:
                cv2.putText(debug, "CAPTURA = TRUE", (10, 135),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

            if self.show_windows:
                cv2.imshow("Mascara naranja ROI", mask)

            return x, y, r, True, capture, error_x_goal, debug, error_x, error_y

        # ==========================
        # SI NO ENCONTRO PELOTA
        # ==========================
        self.lost_frames += 1

        if self.last_center is not None and self.lost_frames < MAX_LOST:
            x, y = self.last_center
            r = self.last_radius if self.last_radius is not None else 10

            cv2.circle(debug, (x, y), r, (0, 255, 255), 1)
            cv2.putText(debug, f"Perdida ({self.lost_frames}/{MAX_LOST})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            self.last_center = None
            self.last_radius = None
            cv2.putText(debug, "No veo pelota naranja", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        if self.show_windows:
            cv2.imshow("Mascara naranja ROI", mask)

        return None, None, None, False, False, error_x_goal, debug, None, None

    def release(self):
        self.cap.release()
        if self.show_windows:
            cv2.destroyAllWindows()
