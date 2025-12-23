from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt5.QtWidgets import QOpenGLWidget, QApplication, QMessageBox
from PyQt5.QtCore import Qt, QPoint, QTimer
from PyQt5.QtGui import QCursor
from PyQt5.QtGui import QImage
import time, os, numpy as np, cv2, queue
from src.modules.plane_model import Uav_Model
from src.modules.static_models import StaticModels
from src.utils.math_utils import *
from config.Config import *
from typing import List, Tuple, Optional

# Kamera modları
CAM_FPS = 0; CAM_FREE = 1; CAM_3RD = 2
CAM_NAMES = {CAM_FPS: "FPS", CAM_FREE: "FREE", CAM_3RD: "3RD"}

# =====================================================================
# FLIGHT PHYSICS ENGINE
# =====================================================================
class FlightPhysics:
    """Gerçek uçak dinamikleri simülasyonu"""
    def __init__(self, mass=1.2):
        self.mass = mass
        self.max_thrust = 15.0
        self.air_density = 1.225
        self.gravity = 9.81
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.drag_coeff = 0.15
        
    def update(self, dt, throttle, control_inputs, rotation):
        """Fizik güncellemesi"""
        # Thrust
        thrust = throttle * self.max_thrust
        
        # Yön vektörü
        pitch_rad = np.radians(rotation[0])
        yaw_rad = np.radians(rotation[1] + 180)
        
        thrust_vec = np.array([
            thrust * np.sin(yaw_rad) * np.cos(pitch_rad),
            -thrust * np.sin(pitch_rad),
            thrust * np.cos(yaw_rad) * np.cos(pitch_rad)
        ])
        
        # Gravity
        gravity_force = np.array([0, -self.mass * self.gravity, 0])
        
        # Drag 
        speed = np.linalg.norm(self.velocity)
        if speed > 0.01:
            drag = -self.velocity * self.drag_coeff * speed
        else:
            drag = np.zeros(3)
        
        # Total force
        total_force = thrust_vec + gravity_force + drag
        
        # Acceleration
        accel = total_force / self.mass
        
        # Update velocity
        self.velocity += accel * dt
        
        # Damping
        self.velocity *= (1.0 - 0.05 * dt)
        
        return self.velocity.copy()


# =====================================================================
# MOTION DETECTOR
# =====================================================================
import os
import sys
import traceback
from typing import List, Tuple, Optional

import numpy as np
import cv2

# PyOpenGL import
try:
    from OpenGL import GL as gl
except Exception as e:
    raise ImportError("PyOpenGL bulunamadı. 'pip install PyOpenGL' ile yükleyin.") from e


# -------------------------
# OpenGL -> BGR okuma fonksiyonları
# -------------------------
# def get_viewport_size() -> Tuple[int, int]:
#     """
#     Aktif GL context içindeki viewport boyutunu (width, height) döndürür.
#     Eğer context yoksa (veya çağrı başarısızsa) RuntimeError fırlatır.
#     """
#     vp = (gl.GLint * 4)()
#     gl.glGetIntegerv(gl.GL_VIEWPORT, vp)
#     x, y, w, h = int(vp[0]), int(vp[1]), int(vp[2]), int(vp[3])
#     if w <= 0 or h <= 0:
#         raise RuntimeError(f"Viewport boyutu geçersiz: {(x,y,w,h)}. Muhtemelen GL context aktif değil.")
#     return w, h


# def gl_read_to_bgr(width: Optional[int] = None,
#                    height: Optional[int] = None,
#                    try_back_first: bool = True,
#                    require_context: bool = True) -> np.ndarray:
#     """
#     Aktif GL context'ten glReadPixels ile güvenli okuma yapar ve OpenCV BGR (uint8) döndürür.
#     - width/height belirtilmezse viewport'tan alınır.
#     - try_back_first: önce GL_BACK deneyip hata alırsa GL_FRONT'ı dener.
#     Raises RuntimeError on failure.
#     """
#     # viewport'tan boyut al (eğer verilmemişse)
#     if width is None or height is None:
#         try:
#             w, h = get_viewport_size()
#         except Exception as e:
#             if require_context:
#                 raise
#             else:
#                 # fallback
#                 w, h = (width or 640), (height or 480)
#         width, height = w, h

#     # Paket hizalamasını 1 yap (satır dolum sorunlarını engellemek için)
#     gl.glPixelStorei(gl.GL_PACK_ALIGNMENT, 1)

#     # Hangi buffer'lar denenir
#     read_bufs = [gl.GL_BACK, gl.GL_FRONT] if try_back_first else [gl.GL_FRONT, gl.GL_BACK]

#     last_exc = None
#     raw = None
#     for rb in read_bufs:
#         try:
#             # bazı implementasyonlarda glReadBuffer() çağrısı hata verebilir -> try/except
#             try:
#                 gl.glReadBuffer(rb)
#             except Exception:
#                 # görmezden gel ve devam et; bazı bağlamlar glReadBuffer'ı desteklemez
#                 pass

#             # glReadPixels çağrısı
#             raw = gl.glReadPixels(0, 0, width, height, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE)
#             # Çıktı alınmışsa döngüden çık
#             last_exc = None
#             break
#         except Exception as e:
#             last_exc = e
#             raw = None
#             continue

#     if raw is None:
#         # hata mesajını detaylı ver
#         msg = "glReadPixels başarısız oldu."
#         if last_exc is not None:
#             msg += f" Son istisna: {last_exc}"
#         raise RuntimeError(msg)

#     # Byte dizisini numpy array'e dönüştür
#     arr = np.frombuffer(raw, dtype=np.uint8)
#     expected_len = width * height * 4
#     if arr.size != expected_len:
#         raise RuntimeError(f"Beklenen uzunluk {expected_len}, alındı {arr.size} - framebuffer formatı farklı olabilir.")

#     # reshape: (height, width, 4). OpenGL bottom-left origin olduğu için flipud ile düzelt
#     arr = arr.reshape((height, width, 4))
#     arr = np.flipud(arr)

#     # RGBA -> BGR (OpenCV)
#     try:
#         bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
#     except Exception as e:
#         raise RuntimeError("RGBA -> BGR dönüşümü sırasında hata: " + str(e)) from e

#     return bgr


# def qwidget_grab_to_bgr(qwidget) -> np.ndarray:
#     """
#     Eğer PyQt/PySide kullanıyorsanız: widget.grabFramebuffer() (QImage) ile daha güvenli okuma yapın.
#     qwidget: QOpenGLWidget veya QWidget (QOpenGLWidget içinde makeCurrent() yapılmış olabilir)
#     Döner: BGR numpy array
#     """
#     try:
#         qimg = qwidget.grabFramebuffer()   # QImage döner
#     except Exception as e:
#         raise RuntimeError("grabFramebuffer başarısız: " + str(e))

#     # QImage -> bytes -> numpy conversion
#     qimg = qimg.convertToFormat(6)  # QImage::Format_RGBA8888 = 6 (PyQt versiyonlarına göre değişebilir)
#     width = qimg.width()
#     height = qimg.height()
#     ptr = qimg.bits()
#     ptr.setsize(qimg.byteCount())
#     arr = np.frombuffer(ptr, np.uint8).reshape((height, width, 4))
#     arr = np.flipud(arr)  # origin düzelt
#     bgr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
#     return bgr


# # -------------------------
# # MotionDetector (OpenCV tarafı)
# # -------------------------
# class MotionDetector:
#     """
#     OpenCV BGR frame bekleyen hareket dedektörü.
#     - use_running_avg: accumulateWeighted ile arka plan modellemesi
#     - resize_width: performans için opsiyonel ölçeklendirme
#     """
#     def __init__(self,
#                  threshold: int = 25,
#                  min_area: int = 800,
#                  use_running_avg: bool = True,
#                  avg_alpha: float = 0.15,
#                  blur_ksize: Tuple[int, int] = (21, 21),
#                  resize_width: Optional[int] = None,
#                  debug: bool = False):
#         self.threshold = threshold
#         self.min_area = min_area
#         self.use_running_avg = use_running_avg
#         self.avg_alpha = float(np.clip(avg_alpha, 0.01, 1.0))
#         self.blur_ksize = blur_ksize
#         self.resize_width = resize_width
#         self.prev_bg = None
#         self.motion_detected = False
#         self.debug = debug
#         self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

#     def _maybe_resize(self, frame: np.ndarray) -> Tuple[np.ndarray, float]:
#         if self.resize_width is None:
#             return frame, 1.0
#         h, w = frame.shape[:2]
#         if w <= self.resize_width:
#             return frame, 1.0
#         scale = self.resize_width / float(w)
#         new_h = int(h * scale)
#         resized = cv2.resize(frame, (self.resize_width, new_h), interpolation=cv2.INTER_AREA)
#         return resized, scale

#     def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, bool, List[Tuple[int,int,int,int]]]:
#         """
#         frame: BGR uint8 numpy array
#         returns: (annotated_frame, motion_detected_bool, list_of_regions)
#         """
#         if frame is None or getattr(frame, "size", 0) == 0:
#             return frame, False, []

#         try:
#             if frame.ndim != 3 or frame.shape[2] != 3:
#                 if self.debug:
#                     print("MotionDetector: Beklenen BGR frame, gelen shape:", frame.shape)
#                 return frame, False, []

#             small, scale = self._maybe_resize(frame)
#             gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
#             gray = cv2.GaussianBlur(gray, self.blur_ksize, 0)

#             if self.prev_bg is None:
#                 self.prev_bg = gray.astype("float")
#                 return frame, False, []

#             if self.use_running_avg:
#                 cv2.accumulateWeighted(gray, self.prev_bg, self.avg_alpha)
#                 bg_uint8 = cv2.convertScaleAbs(self.prev_bg)
#                 frame_delta = cv2.absdiff(bg_uint8, gray)
#             else:
#                 prev_uint8 = cv2.convertScaleAbs(self.prev_bg)
#                 frame_delta = cv2.absdiff(prev_uint8, gray)
#                 self.prev_bg = gray.astype("float")

#             _, thresh = cv2.threshold(frame_delta, self.threshold, 255, cv2.THRESH_BINARY)
#             thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, self._kernel, iterations=1)
#             thresh = cv2.dilate(thresh, None, iterations=2)

#             contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             motion_regions = []

#             for cnt in contours:
#                 area = cv2.contourArea(cnt)
#                 if area < self.min_area:
#                     continue
#                 x, y, w, h = cv2.boundingRect(cnt)
#                 if scale != 1.0:
#                     inv_scale = 1.0 / scale
#                     x = int(x * inv_scale)
#                     y = int(y * inv_scale)
#                     w = int(w * inv_scale)
#                     h = int(h * inv_scale)
#                 motion_regions.append((x, y, w, h))

#             self.motion_detected = len(motion_regions) > 0

#             result = frame.copy()
#             for (x, y, w, h) in motion_regions:
#                 cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

#             status = "MOTION DETECTED" if self.motion_detected else "No Motion"
#             color = (0, 255, 0) if self.motion_detected else (200, 200, 200)
#             cv2.putText(result, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
#             cv2.putText(result, f"Regions: {len(motion_regions)}", (10, 55),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

#             return result, self.motion_detected, motion_regions

#         except Exception:
#             if self.debug:
#                 traceback.print_exc()
#             self.prev_bg = None
#             return frame, False, []

#     def reset(self):
#         self.prev_bg = None

#     def encode_frame(self, frame: np.ndarray, ext: str = ".jpg", quality: int = 80) -> bytes:
#         if frame is None:
#             return b""
#         encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(quality, 10, 95))]
#         success, buf = cv2.imencode(ext, frame, encode_param)
#         if not success:
#             return b""
#         return buf.tobytes()
# class MotionDetector:
#     """OpenCV ile hareket tespiti"""
#     def __init__(self, threshold=30, min_area=800):
#         self.threshold = threshold
#         self.min_area = min_area
#         self.prev_frame = None
#         self.motion_detected = False
        
#     def detect(self, frame):
#         """Hareket tespit et"""
#         if frame is None or frame.size == 0:
#             return frame, False, []
        
#         try:
#             # Boyut kontrolü
#             if len(frame.shape) != 3 or frame.shape[2] != 3:
#                 return frame, False, []
            
#             h, w = frame.shape[:2]
#             if h < 10 or w < 10:
#                 return frame, False, []
            
#             # Gri tonlama
#             gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
#             gray = cv2.GaussianBlur(gray, (21, 21), 0)
            
#             # İlk frame
#             if self.prev_frame is None or self.prev_frame.shape != gray.shape:
#                 self.prev_frame = gray.copy()
#                 return frame, False, []
            
#             # Frame farkı
#             frame_delta = cv2.absdiff(self.prev_frame, gray)
#             thresh = cv2.threshold(frame_delta, self.threshold, 255, cv2.THRESH_BINARY)[1]
#             thresh = cv2.dilate(thresh, None, iterations=2)
            
#             # Konturlar
#             contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
#             motion_regions = []
#             result = frame.copy()
            
#             for contour in contours:
#                 if cv2.contourArea(contour) < self.min_area:
#                     continue
                
#                 x, y, w, h = cv2.boundingRect(contour)
#                 motion_regions.append((x, y, w, h))
#                 cv2.rectangle(result, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
#             self.motion_detected = len(motion_regions) > 0
            
#             # Status text
#             status = "MOTION DETECTED" if self.motion_detected else "No Motion"
#             color = (0, 255, 0) if self.motion_detected else (200, 200, 200)
#             cv2.putText(result, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
#             cv2.putText(result, f"Regions: {len(motion_regions)}", (10, 55), 
#                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
#             self.prev_frame = gray.copy()
#             return result, self.motion_detected, motion_regions
            
#         except Exception as e:
#             print(f"Motion detection error: {e}")
#             self.prev_frame = None
#             return frame, False, []
    
#     def reset(self):
#         self.prev_frame = None
import collections
import time
from typing import List, Tuple, Optional

import cv2
import numpy as np


class CentroidTracker:
    """
    Basit centroid tracker: ID atar, kaybolma sayacı tutar.
    Only used to provide persistence/confirmation of blobs.
    """
    def __init__(self, max_disappeared=10, max_distance=50):
        self.next_object_id = 0
        self.objects = dict()  # id -> centroid (x,y)
        self.disappeared = dict()  # id -> frames disappeared
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def register(self, centroid):
        oid = self.next_object_id
        self.objects[oid] = centroid
        self.disappeared[oid] = 0
        self.next_object_id += 1
        return oid

    def deregister(self, oid):
        if oid in self.objects:
            del self.objects[oid]
        if oid in self.disappeared:
            del self.disappeared[oid]

    def update(self, rects: List[Tuple[int,int,int,int]]):
        """
        rects: list of (x,y,w,h)
        returns dict of id -> centroid
        """
        if len(rects) == 0:
            # increment disappeared counters
            for oid in list(self.disappeared.keys()):
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)
            return self.objects.copy()

        input_centroids = []
        for (x, y, w, h) in rects:
            cX = int(x + w / 2.0)
            cY = int(y + h / 2.0)
            input_centroids.append((cX, cY))

        if len(self.objects) == 0:
            for c in input_centroids:
                self.register(c)
            return self.objects.copy()

        # match existing object centroids to new centroids via distance matrix
        object_ids = list(self.objects.keys())
        object_centroids = list(self.objects.values())

        D = np.linalg.norm(np.array(object_centroids)[:, None, :] - np.array(input_centroids)[None, :, :], axis=2)
        # rows: existing objects, cols: inputs
        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]

        used_rows = set()
        used_cols = set()

        for (r, c) in zip(rows, cols):
            if r in used_rows or c in used_cols:
                continue
            if D[r, c] > self.max_distance:
                continue
            oid = object_ids[r]
            self.objects[oid] = input_centroids[c]
            self.disappeared[oid] = 0
            used_rows.add(r)
            used_cols.add(c)

        # unmatched existing objects -> disappeared++
        unused_rows = set(range(0, D.shape[0])).difference(used_rows)
        for r in unused_rows:
            oid = object_ids[r]
            self.disappeared[oid] += 1
            if self.disappeared[oid] > self.max_disappeared:
                self.deregister(oid)

        # unmatched new centroids -> register
        unused_cols = set(range(0, D.shape[1])).difference(used_cols)
        for c in unused_cols:
            self.register(input_centroids[c])

        return self.objects.copy()


class MotionDetector:
    """
    Profesyonel, AI içermeyen hareket dedektörü:
      - BackgroundSubtractorMOG2 (cv2) ile foreground mask
      - Shadow removal (MOG2 shadow value = 127)
      - Morphological cleaning (open/close)
      - Temporal accumulation (deque of masks) ile persistence filtering
      - Contour filtering by area + optional aspect/solidity filters
      - Basit CentroidTracker ile doğrulama (opsiyonel)
    Beklenen frame format: BGR uint8 (OpenCV standard)
    """

    def __init__(self,
                 min_area: int = 800,
                 resize_width: Optional[int] = None,
                 history: int = 500,
                 var_threshold: int = 16,
                 detect_shadows: bool = True,
                 morph_kernel: int = 3,
                 blur_ksize: Tuple[int, int] = (5, 5),
                 persistence_frames: int = 3,
                 tracker_confirm_frames: int = 2,
                 debug: bool = False):
        # Sensitivity / filtering
        self.min_area = min_area
        self.resize_width = resize_width
        self.blur_ksize = blur_ksize
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph_kernel, morph_kernel))
        self.persistence_frames = max(1, persistence_frames)  # how many recent masks to require
        self.tracker_confirm_frames = max(1, tracker_confirm_frames)
        self.debug = debug

        # Background subtractor
        self.backsub = cv2.createBackgroundSubtractorMOG2(history=history,
                                                          varThreshold=var_threshold,
                                                          detectShadows=detect_shadows)
        self.detect_shadows = detect_shadows

        # Temporal mask history (deque)
        self._mask_history = collections.deque(maxlen=self.persistence_frames)

        # Simple tracker to confirm objects across frames
        self._tracker = CentroidTracker(max_disappeared=10, max_distance=50)
        self._object_seen_frames = dict()  # object_id -> times seen (for confirmation)

        # state
        self.motion_detected = False

    def _maybe_resize(self, frame: np.ndarray):
        if self.resize_width is None:
            return frame, 1.0
        h, w = frame.shape[:2]
        if w <= self.resize_width:
            return frame, 1.0
        scale = self.resize_width / float(w)
        new_h = int(h * scale)
        resized = cv2.resize(frame, (self.resize_width, new_h), interpolation=cv2.INTER_AREA)
        return resized, scale

    def _clean_mask(self, mask: np.ndarray):
        # remove shadows (127) if MOG2 shadows enabled
        if self.detect_shadows:
            # MOG2: shadow value == 127
            mask = np.where(mask == 127, 0, mask).astype('uint8')

        # threshold to binary (ensure 0 or 255)
        _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

        # small blur then morphology
        if self.blur_ksize is not None and (self.blur_ksize[0] > 1 or self.blur_ksize[1] > 1):
            mask = cv2.GaussianBlur(mask, self.blur_ksize, 0)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        return mask

    def _temporal_persistence(self, mask: np.ndarray):
        """
        Keep last N masks, require pixel to be present in >= ceil(N/2) frames.
        This reduces flicker / false positives.
        """
        self._mask_history.append(mask)
        if len(self._mask_history) == 0:
            return mask
        # sum masks (255 * count), convert to counts
        acc = np.sum(np.stack([m // 255 for m in self._mask_history], axis=0), axis=0)
        required = int(np.ceil(len(self._mask_history) / 2.0))
        stable = (acc >= required).astype('uint8') * 255
        return stable

    def detect(self, frame: np.ndarray) -> Tuple[np.ndarray, bool, List[Tuple[int,int,int,int]]]:
        """
        Input: BGR uint8 frame
        Output: annotated_frame (BGR), detected(bool), regions list[(x,y,w,h)]
        """
        if frame is None or getattr(frame, "size", 0) == 0:
            return frame, False, []

        try:
            # Validate shape
            if frame.ndim != 3 or frame.shape[2] != 3:
                if self.debug:
                    print("MotionDetector: Unexpected frame shape:", getattr(frame, "shape", None))
                return frame, False, []

            # resize for speed if requested
            small, scale = self._maybe_resize(frame)

            # background subtractor -> foreground mask
            fgmask = self.backsub.apply(small)

            # clean mask (shadow removal + morphology + blur)
            clean = self._clean_mask(fgmask)

            # temporal persistence
            stable = self._temporal_persistence(clean)

            # find contours on stable mask
            contours, _ = cv2.findContours(stable, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            motion_regions = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.min_area:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                # scale back to original frame coordinates if resized
                if scale != 1.0:
                    inv = 1.0 / scale
                    x = int(x * inv)
                    y = int(y * inv)
                    w = int(w * inv)
                    h = int(h * inv)
                motion_regions.append((x, y, w, h))

            # update tracker and confirm persistent objects
            confirmed_regions = []
            if len(motion_regions) > 0:
                objects = self._tracker.update(motion_regions)
                # increment seen frames for matched objects
                current_ids = set(objects.keys())
                for oid in list(self._object_seen_frames.keys()):
                    if oid not in current_ids:
                        self._object_seen_frames[oid] = max(0, self._object_seen_frames[oid] - 1)
                for oid, centroid in objects.items():
                    self._object_seen_frames.setdefault(oid, 0)
                    self._object_seen_frames[oid] += 1
                # find objects that reached confirmation threshold and map to bounding boxes
                # mapping centroid -> rect: choose nearest rect in motion_regions
                for oid, centroid in objects.items():
                    if self._object_seen_frames.get(oid, 0) >= self.tracker_confirm_frames:
                        # find nearest rect center
                        best_rect = None
                        best_dist = float('inf')
                        for rect in motion_regions:
                            rx, ry, rw, rh = rect
                            rc = (int(rx + rw/2), int(ry + rh/2))
                            d = (rc[0]-centroid[0])**2 + (rc[1]-centroid[1])**2
                            if d < best_dist:
                                best_dist = d
                                best_rect = rect
                        if best_rect is not None:
                            confirmed_regions.append(best_rect)
            else:
                # no motion; update tracker (clearing/disappearing handled inside)
                _ = self._tracker.update([])

            self.motion_detected = len(confirmed_regions) > 0

            # annotate on original frame
            annotated = frame.copy()
            for (x, y, w, h) in confirmed_regions:
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)

            status = "MOTION DETECTED" if self.motion_detected else "No Motion"
            color = (0, 255, 0) if self.motion_detected else (200, 200, 200)
            cv2.putText(annotated, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(annotated, f"Regions: {len(confirmed_regions)}", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            return annotated, self.motion_detected, confirmed_regions

        except Exception as e:
            if self.debug:
                import traceback
                traceback.print_exc()
            # reset some state if catastrophic error
            self._mask_history.clear()
            self._object_seen_frames.clear()
            self._tracker = CentroidTracker(max_disappeared=10, max_distance=50)
            self.motion_detected = False
            return frame, False, []

    def reset(self):
        self._mask_history.clear()
        self._object_seen_frames.clear()
        self._tracker = CentroidTracker(max_disappeared=10, max_distance=50)
        self.backsub = cv2.createBackgroundSubtractorMOG2()
        self.motion_detected = False


# =====================================================================
# OPENGL WIDGET
# =====================================================================
class OpenGLWidget(QOpenGLWidget):
    def __init__(self):
        super().__init__()
        self.Plane = Uav_Model()
        self.physics = FlightPhysics()
        self.motion_detector = MotionDetector()
        self._output_queue = queue.Queue()
        
        # Timing
        self.last_time = time.time()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.on_frame)
        self.timer.start(16)
        
        # Camera
        self.camera_mode = CAM_FPS
        self.keys_down = set()
        self.free_pos = [0.0, 1.5, 0.0]
        self.free_rot = [0.0, 0.0]
        
        # Mouse
        self.mouse_sensitivity = 0.01
        self.move_speed = 3.5
        self.mouse_captured = False
        self.mouse_x_inverted = True
        self.mouse_y_inverted = False
        
        # Models
        self.static_models = []
        self.model_vertices = None
        self.model_texcoords = None
        self.model_count = 0
        self.vbo_vertices = None
        self.vbo_texcoords = None
        self.texture_id = None
        self.use_vbo = True
        
        # Motion detection
        self.show_motion_detection = True
        self.motion_window_created = False
        
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)
    
    def initializeGL(self):
        try:
            glClearColor(0.07, 0.09, 0.15, 1.0)
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_CULL_FACE)
            glEnable(GL_MULTISAMPLE)
            self._load_default_models()
        except Exception as e:
            QMessageBox.critical(None, "GL Init Error", str(e))
    
    def _load_default_models(self):
        """Modelleri yükle"""
        try:
            # Drone
            if os.path.isfile(DEFAULT_MODEL_PATH):
                tex = DEFAULT_TEXTURE_PATH if os.path.isfile(DEFAULT_TEXTURE_PATH) else None
                self.load_model_with_texture(DEFAULT_MODEL_PATH, tex)
                print("✓ Drone model loaded")
            
            # Ground
            if os.path.isfile(DEFAULT_LAXTON_PATH):
                gid = self.add_ground_model(DEFAULT_LAXTON_PATH, [0, 1, 300], [0, 270, -0])
                self.load_static_model(gid, DEFAULT_LAXTON_TEXTURE_PATH)
                print("✓ Ground loaded")
            
            # Road
            if os.path.isfile(DEFAULT_ROAD_PATH):
                rid = self.add_building_model(DEFAULT_ROAD_PATH, [0, -2, 7], [0, 90, 0])
                self.load_static_model(rid, DEFAULT_ROAD_TEXTURE_PATH)
                print("✓ Road loaded")
            
            # Mosque
            if os.path.isfile(DEFAULT_MOSQUE_PATH):
                mid = self.add_building_model(DEFAULT_MOSQUE_PATH, [-8, 0.5, 8], [-10, -20, -3])
                self.load_static_model(mid, DEFAULT_MOSQUE_TEXTURE_PATH)
                print("✓ Mosque loaded")
                
        except Exception as e:
            print(f"Model loading error: {e}")
            import traceback
            traceback.print_exc()
    
    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w/h, 0.1, 500.0)
        glMatrixMode(GL_MODELVIEW)
    
    def on_frame(self):
        """Frame update"""
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        if self.camera_mode == CAM_FREE:
            self._update_free_camera(dt)
        else:
            self._update_flight(dt)
            # Velocity ile pozisyon güncelleme (orijinal koddan)
            if hasattr(self.Plane, 'velocity') and any(abs(v) > 1e-9 for v in self.Plane.velocity):
                self.Plane.position[0] += self.Plane.velocity[0] * dt
                self.Plane.position[1] += self.Plane.velocity[1] * dt
                self.Plane.position[2] += self.Plane.velocity[2] * dt
        
        # Motion detection
        if self.camera_mode == CAM_FPS and self.show_motion_detection:
            self._capture_and_detect()
        
        self.Plane._save_if_changed()
        self.update()
    
    def _update_free_camera(self, dt):
        """Free camera hareketi"""
        forward = right = upv = 0.0
        if Qt.Key_W in self.keys_down: forward += 1.0
        if Qt.Key_S in self.keys_down: forward -= 1.0
        if Qt.Key_D in self.keys_down: right -= 1.0
        if Qt.Key_A in self.keys_down: right += 1.0
        if Qt.Key_Space in self.keys_down: upv += 1.0
        if Qt.Key_Control in self.keys_down: upv -= 1.0
        
        yaw = np.radians(self.free_rot[1] + 180)
        forward_vec = (np.sin(yaw) * forward, 0.0, np.cos(yaw) * forward)
        right_vec = (np.cos(yaw) * right, 0.0, -np.sin(yaw) * right)
        
        dx = (forward_vec[0] + right_vec[0]) * self.move_speed * dt
        dy = upv * self.move_speed * dt
        dz = (forward_vec[2] + right_vec[2]) * self.move_speed * dt
        
        self.free_pos[0] += dx
        self.free_pos[1] += dy
        self.free_pos[2] += dz
    
    def _update_flight(self, dt):
        """Uçuş fiziği - ORİJİNAL KONTROLLER"""
        # Kuvvet hesaplama
        forward = right = upv = 0.0
        if Qt.Key_W in self.keys_down: forward += 1.0
        if Qt.Key_S in self.keys_down: forward -= 1.0
        if Qt.Key_D in self.keys_down: right -= 1.0
        if Qt.Key_A in self.keys_down: right += 1.0
        if Qt.Key_Space in self.keys_down: upv += 1.0
        if Qt.Key_Control in self.keys_down: upv -= 1.0
        
        yaw = np.radians(self.Plane.rotation[1] + 180)
        forward_vec = (np.sin(yaw) * forward, 0.0, np.cos(yaw) * forward)
        right_vec = (np.cos(yaw) * right, 0.0, -np.sin(yaw) * right)
        
        # Kuvvet vektörleri
        force_x = forward_vec[0] + right_vec[0]
        force_z = forward_vec[2] + right_vec[2]
        
        # Fizik parametreleri
        mass = getattr(self.Plane, 'mass', 1.0)
        friction = getattr(self.Plane, 'friction', 0.1)
        max_force = getattr(self.Plane, 'max_force', 10.0)
        
        # Kuvvet sınırlama
        total_horizontal_force = (force_x**2 + force_z**2)**0.5
        if total_horizontal_force > max_force:
            force_x = force_x * max_force / total_horizontal_force
            force_z = force_z * max_force / total_horizontal_force
        
        # Hız güncellemesi
        if hasattr(self.Plane, 'velocity'):
            # Sürtünme
            friction_force_x = -self.Plane.velocity[0] * friction
            friction_force_z = -self.Plane.velocity[2] * friction
            
            # Toplam kuvvet
            total_force_x = force_x * self.move_speed + friction_force_x
            total_force_z = force_z * self.move_speed + friction_force_z
            
            # İvme (F = ma -> a = F/m)
            accel_x = total_force_x / mass
            accel_z = total_force_z / mass
            
            # Hız güncellemesi
            self.Plane.velocity[0] += accel_x * dt
            self.Plane.velocity[2] += accel_z * dt
        
        # Dikey hareket
        dy = upv * self.move_speed * dt
        self.Plane.translate(0, dy, 0)
        
        self.Plane._save_if_changed()
    
    def _capture_and_detect(self):
        """
        QOpenGLWidget için DOĞRU yol:
        glReadPixels / glReadBuffer KULLANILMAZ
        """
        try:
            # Qt framebuffer yakalama
            qimg = self.grabFramebuffer()

            if qimg.isNull():
                return

            # RGBA8888 garanti
            qimg = qimg.convertToFormat(QImage.Format_RGBA8888)

            w = qimg.width()
            h = qimg.height()

            ptr = qimg.bits()
            ptr.setsize(qimg.byteCount())

            frame = np.frombuffer(ptr, np.uint8).reshape((h, w, 4))

            # Qt top-left → OpenCV top-left (flip GEREKMİYOR)
            frame = frame.copy()  # Qt buffer ömrü için

            # RGBA → BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

            processed, detected, regions = self.motion_detector.detect(frame)

            if processed is not None:
                cv2.imshow("Motion Detection", processed)
                cv2.waitKey(1)
                self.motion_window_created = True

        except Exception as e:
            print("Motion detection error:", e)
            
    def keyPressEvent(self, ev):
        k = ev.key()
        
        if k == Qt.Key_M:
            self.camera_mode = (self.camera_mode + 1) % 3
            if self.camera_mode != CAM_FPS:
                self.motion_detector.reset()
                if self.motion_window_created:
                    cv2.destroyWindow("Motion Detection")
                    self.motion_window_created = False
            self._update_title()
            return
        
        if k == Qt.Key_Q:
            self.toggle_mouse_capture()
            return
        
        if k == Qt.Key_V:
            self.show_motion_detection = not self.show_motion_detection
            if not self.show_motion_detection and self.motion_window_created:
                cv2.destroyWindow("Motion Detection")
                self.motion_window_created = False
            print(f"Motion Detection: {'ON' if self.show_motion_detection else 'OFF'}")
            return
        
        self.keys_down.add(k)
    
    def keyReleaseEvent(self, ev):
        if ev.key() in self.keys_down:
            self.keys_down.remove(ev.key())
    
    def toggle_mouse_capture(self):
        """Mouse capture toggle - Q tuşu"""
        self.mouse_captured = not self.mouse_captured
        if self.mouse_captured:
            QApplication.setOverrideCursor(Qt.BlankCursor)
            self._center_cursor()
        else:
            try:
                QApplication.restoreOverrideCursor()
            except:
                pass
        self._update_title()
    
    def mouseMoveEvent(self, ev):
        if not self.mouse_captured:
            return super().mouseMoveEvent(ev)
        
        center = QPoint(self.width() // 2, self.height() // 2)
        dx = ev.x() - center.x()
        dy = ev.y() - center.y()
        
        sign_x = -1.0 if self.mouse_x_inverted else 1.0
        sign_y = -1.0 if self.mouse_y_inverted else 1.0
        
        yaw_delta = sign_x * dx * self.mouse_sensitivity
        pitch_delta = sign_y * dy * self.mouse_sensitivity
        
        yaw_delta = np.clip(yaw_delta, -8, 8)
        pitch_delta = np.clip(pitch_delta, -8, 8)
        
        if self.camera_mode == CAM_FREE:
            self.free_rot[1] += yaw_delta
            self.free_rot[0] = np.clip(self.free_rot[0] + pitch_delta, -89.9, 89.9)
        else:
            self.Plane.rotation[1] += yaw_delta
            self.Plane.rotation[0] = np.clip(self.Plane.rotation[0] + pitch_delta, -89.9, 89.9)
        
        self._center_cursor()
        super().mouseMoveEvent(ev)
    
    def _center_cursor(self):
        center = self.mapToGlobal(QPoint(self.width()//2, self.height()//2))
        QCursor.setPos(center)
    
    def _update_title(self):
        top = self.window()
        if top:
            mode = CAM_NAMES[self.camera_mode]
            cap = "CAPTURED" if self.mouse_captured else "FREE"
            top.setWindowTitle(f"Flight Sim - {mode} - Mouse: {cap}")
    
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        eye, target, up = self._compute_camera()
        gluLookAt(*eye, *target, *up)
        
        self._draw_grid()
        self._draw_static_models()
        self._draw_drone()
    
    def _draw_static_models(self):
        for m in self.static_models:
            if not m.loaded:
                continue
            glPushMatrix()
            glTranslatef(*m.position)
            glRotatef(m.rotation[0], 1, 0, 0)
            glRotatef(m.rotation[1], 0, 1, 0)
            glRotatef(m.rotation[2], 0, 0, 1)
            glScalef(*m.scale)
            self._draw_static_model(m)
            glPopMatrix()
    
    def _draw_drone(self):
        glPushMatrix()
        glTranslatef(*self.Plane.position)
        glRotatef(self.Plane.rotation[0], 1, 0, 0)
        glRotatef(self.Plane.rotation[1], 0, 1, 0)
        glRotatef(self.Plane.rotation[2], 0, 0, 1)
        glScalef(*self.Plane.scale)
        
        if self.model_vertices is not None:
            self._draw_model_vbo()
        else:
            self._draw_cube(self.Plane.color)
        
        glPopMatrix()
    
    def _draw_model_vbo(self):
        if not self.vbo_vertices:
            return
        
        if self.texture_id:
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, self.texture_id)
        
        glEnableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_vertices)
        glVertexPointer(3, GL_FLOAT, 0, None)
        
        if self.vbo_texcoords:
            glEnableClientState(GL_TEXTURE_COORD_ARRAY)
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo_texcoords)
            glTexCoordPointer(2, GL_FLOAT, 0, None)
        
        glColor3f(*self.Plane.color)
        glDrawArrays(GL_TRIANGLES, 0, self.model_count)
        
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDisableClientState(GL_VERTEX_ARRAY)
        if self.vbo_texcoords:
            glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        if self.texture_id:
            glDisable(GL_TEXTURE_2D)
    
    def _draw_static_model(self, m):
        if not m.vbo_vertices:
            return
        
        glEnableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, m.vbo_vertices)
        glVertexPointer(3, GL_FLOAT, 0, None)
        
        if m.vbo_texcoords and m.texture_id:
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, m.texture_id)
            glEnableClientState(GL_TEXTURE_COORD_ARRAY)
            glBindBuffer(GL_ARRAY_BUFFER, m.vbo_texcoords)
            glTexCoordPointer(2, GL_FLOAT, 0, None)
        
        glColor3f(*m.color)
        glDrawArrays(GL_TRIANGLES, 0, m.vertex_count)
        
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDisableClientState(GL_VERTEX_ARRAY)
        if m.vbo_texcoords:
            glDisableClientState(GL_TEXTURE_COORD_ARRAY)
            glDisable(GL_TEXTURE_2D)
    
    def _draw_cube(self, color):
        r, g, b = color
        glBegin(GL_QUADS)
        # Front
        glColor3f(r, g, b)
        glVertex3f(-0.5,-0.5,0.5); glVertex3f(0.5,-0.5,0.5)
        glVertex3f(0.5,0.5,0.5); glVertex3f(-0.5,0.5,0.5)
        # Back
        glColor3f(r*0.9, g*0.9, b*0.9)
        glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,0.5,-0.5)
        glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,-0.5,-0.5)
        # Left
        glColor3f(r*0.8, g*0.8, b*0.8)
        glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,-0.5,0.5)
        glVertex3f(-0.5,0.5,0.5); glVertex3f(-0.5,0.5,-0.5)
        # Right
        glColor3f(r, g, b)
        glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,0.5,-0.5)
        glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,-0.5,0.5)
        # Top
        glColor3f(r*0.95, g*0.95, b*0.95)
        glVertex3f(-0.5,0.5,-0.5); glVertex3f(-0.5,0.5,0.5)
        glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,0.5,-0.5)
        # Bottom
        glColor3f(r*0.7, g*0.7, b*0.7)
        glVertex3f(-0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,-0.5)
        glVertex3f(0.5,-0.5,0.5); glVertex3f(-0.5,-0.5,0.5)
        glEnd()
    
    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        glColor3f(0.2, 0.2, 0.2)
        glBegin(GL_LINES)
        for i in range(-10, 11):
            glVertex3f(i, -1.0, -10); glVertex3f(i, -1.0, 10)
            glVertex3f(-10, -1.0, i); glVertex3f(10, -1.0, i)
        glEnd()
    
    def _compute_camera(self):
        up = (0.0, 1.0, 0.0)
        
        if self.camera_mode == CAM_FPS:
            eye_offset = (0.0, 0.2, 0.0)
            eye = (
                self.Plane.position[0] + eye_offset[0],
                self.Plane.position[1] + eye_offset[1],
                self.Plane.position[2] + eye_offset[2]
            )
            pitch = self.Plane.rotation[0]
            yaw = self.Plane.rotation[1] + 180.0
            forward = euler_forward(pitch, yaw)
            target = (eye[0] + forward[0], eye[1] + forward[1], eye[2] + forward[2])
            return eye, target, up
        
        elif self.camera_mode == CAM_FREE:
            eye = tuple(self.free_pos)
            pitch = self.free_rot[0]
            yaw = self.free_rot[1] + 180.0
            forward = euler_forward(pitch, yaw)
            target = (eye[0] + forward[0], eye[1] + forward[1], eye[2] + forward[2])
            return eye, target, up
        
        else:  # CAM_3RD - FOLLOW CAMERA
            distance = 4.0
            height = 1.5
            yaw = np.radians(self.Plane.rotation[1] + 180.0)
            offset_x = -np.sin(yaw) * distance
            offset_z = -np.cos(yaw) * distance
            eye = (
                self.Plane.position[0] + offset_x,
                self.Plane.position[1] + height,
                self.Plane.position[2] + offset_z
            )
            target = (
                self.Plane.position[0],
                self.Plane.position[1] + 0.5,
                self.Plane.position[2]
            )
            return eye, target, up
    
    # MODEL LOADING
    def load_model_with_texture(self, obj_path, tex_path=None):
        vert_flat, uv_flat = self.parse_obj_file(obj_path)
        if vert_flat is None:
            return False
        
        self._create_vbos(vert_flat, uv_flat)
        
        try:
            self.Plane.set_rotation(*DEFAULT_MODEL_ROTATION)
        except:
            pass
        
        if tex_path and os.path.isfile(tex_path):
            self.load_texture(tex_path)
        
        return True
    # obj dosyasını parse ediyoruz

    def parse_obj_file(self, path):
        verts = []
        uvs = []
        faces = []
        
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split()
                    if not parts:
                        continue
                    
                    if parts[0] == 'v' and len(parts) >= 4:
                        verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
                    elif parts[0] == 'vt' and len(parts) >= 3:
                        uvs.append((float(parts[1]), float(parts[2])))
                    elif parts[0] == 'f' and len(parts) >= 4:
                        idxs = []
                        uv_idxs = []
                        for comp in parts[1:]:
                            comps = comp.split('/')
                            vi = int(comps[0])
                            if vi < 0:
                                vi = len(verts) + 1 + vi
                            idxs.append(vi - 1)
                            
                            if len(comps) >= 2 and comps[1]:
                                vti = int(comps[1])
                                if vti < 0:
                                    vti = len(uvs) + 1 + vti
                                uv_idxs.append(vti - 1)
                            else:
                                uv_idxs.append(None)
                        
                        for i in range(1, len(idxs) - 1):
                            faces.append(((idxs[0], uv_idxs[0]), (idxs[i], uv_idxs[i]), (idxs[i+1], uv_idxs[i+1])))
        except Exception as e:
            print(f"OBJ parse error: {e}")
            return None, None
        
        vert_flat = []
        uv_flat = []
        for tri in faces:
            for vi, vti in tri:
                x, y, z = verts[vi]
                vert_flat.extend([x, y, z])
                if vti is not None and 0 <= vti < len(uvs):
                    u, v = uvs[vti]
                    uv_flat.extend([u, v])
                else:
                    uv_flat.extend([0, 0])
        
        return vert_flat, uv_flat if uv_flat else None
    
    def _create_vbos(self, vert_array, uv_array):
        vbuf = np.array(vert_array, dtype=np.float32)
        self.model_vertices = vbuf
        self.model_count = len(vert_array) // 3
        
        try:
            if self.vbo_vertices:
                glDeleteBuffers(1, [int(self.vbo_vertices)])
            if self.vbo_texcoords:
                glDeleteBuffers(1, [int(self.vbo_texcoords)])
        except:
            pass
        
        self.vbo_vertices = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, int(self.vbo_vertices))
        glBufferData(GL_ARRAY_BUFFER, vbuf.nbytes, vbuf, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        if uv_array:
            tbuf = np.array(uv_array, dtype=np.float32)
            self.model_texcoords = tbuf
            self.vbo_texcoords = glGenBuffers(1)
            glBindBuffer(GL_ARRAY_BUFFER, int(self.vbo_texcoords))
            glBufferData(GL_ARRAY_BUFFER, tbuf.nbytes, tbuf, GL_STATIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
    
    def load_texture(self, img_path):
        try:
            img = QImage(img_path)
            if img.isNull():
                return False
            
            img = img.convertToFormat(QImage.Format_RGBA8888)
            img = img.mirrored(False, True)
            
            w, h = img.width(), img.height()
            ptr = img.bits()
            ptr.setsize(img.byteCount())
            data = bytes(ptr)
            
            if self.texture_id is None:
                self.texture_id = glGenTextures(1)
            
            glBindTexture(GL_TEXTURE_2D, int(self.texture_id))
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
            
            try:
                glGenerateMipmap(GL_TEXTURE_2D)
            except:
                pass
            
            glBindTexture(GL_TEXTURE_2D, 0)
            return True
        except Exception as e:
            print(f"Texture load error: {e}")
            return False
    
    # STATIC MODELS
    def add_building_model(self, obj_path, position, rotation=None, scale=None):
        building = StaticModels(obj_path, position, rotation or [0,0,0], scale or [1,1,1])
        building.color = (0.6, 0.5, 0.4)
        self.static_models.append(building)
        return len(self.static_models) - 1
    
    def add_ground_model(self, obj_path, position=None, rotation=None, scale=None):
        ground = StaticModels(obj_path, position or [0,-2,0], rotation or [0,0,0], scale or [10,1,10])
        ground.color = (1, 1, 1)
        self.static_models.append(ground)
        return len(self.static_models) - 1
    
    def load_static_model(self, model_id, tex_path=None):
        if model_id >= len(self.static_models):
            return False
        
        m = self.static_models[model_id]
        vert_flat, uv_flat = self.parse_obj_file(m.obj_path)
        
        if vert_flat is None:
            return False
        
        m.vertices = np.array(vert_flat, dtype=np.float32)
        m.vertex_count = len(vert_flat) // 3
        
        if uv_flat:
            uv_arr = np.array(uv_flat, dtype=np.float32)
            expected = m.vertex_count * 2
            if uv_arr.size == expected:
                m.texcoords = uv_arr
            else:
                m.texcoords = None
        else:
            m.texcoords = None
        
        # VBOs
        try:
            vbo_v = glGenBuffers(1)
            m.vbo_vertices = int(vbo_v) if isinstance(vbo_v, (int, np.integer)) else int(vbo_v[0])
            glBindBuffer(GL_ARRAY_BUFFER, m.vbo_vertices)
            glBufferData(GL_ARRAY_BUFFER, m.vertices.nbytes, m.vertices, GL_STATIC_DRAW)
            
            if m.texcoords is not None:
                vbo_t = glGenBuffers(1)
                m.vbo_texcoords = int(vbo_t) if isinstance(vbo_t, (int, np.integer)) else int(vbo_t[0])
                glBindBuffer(GL_ARRAY_BUFFER, m.vbo_texcoords)
                glBufferData(GL_ARRAY_BUFFER, m.texcoords.nbytes, m.texcoords, GL_STATIC_DRAW)
            
            glBindBuffer(GL_ARRAY_BUFFER, 0)
        except Exception as e:
            print(f"Static VBO error: {e}")
            return False
        
        # Texture
        if tex_path and os.path.isfile(tex_path):
            self._load_texture_for_model(tex_path, m)
        
        # Bounding box
        self._calc_bbox(m)
        m.loaded = True
        return True
    
    def _load_texture_for_model(self, tex_path, model):
        try:
            img = QImage(tex_path)
            if img.isNull():
                return False
            
            try:
                img = img.convertToFormat(QImage.Format_RGBA8888)
                need_swap = False
            except:
                img = img.convertToFormat(QImage.Format_ARGB32)
                need_swap = True
            
            w, h = img.width(), img.height()
            ptr = img.bits()
            ptr.setsize(img.byteCount())
            img_data = np.array(ptr, dtype=np.uint8).reshape((h, w, 4))
            
            if need_swap:
                img_data = img_data[..., [2,1,0,3]]
            
            img_data = np.flip(img_data, axis=0)
            img_data = np.ascontiguousarray(img_data, dtype=np.uint8)
            
            tex_id = glGenTextures(1)
            tex_id = int(tex_id) if isinstance(tex_id, (int, np.integer)) else int(tex_id[0])
            
            glBindTexture(GL_TEXTURE_2D, tex_id)
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, img_data)
            glGenerateMipmap(GL_TEXTURE_2D)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
            glBindTexture(GL_TEXTURE_2D, 0)
            
            model.texture_id = tex_id
            return True
        except Exception as e:
            print(f"Model texture error: {e}")
            return False
    
    def _calc_bbox(self, model):
        if model.vertices is None or len(model.vertices) < 3:
            return
        
        verts = model.vertices.reshape(-1, 3)
        min_c = np.min(verts, axis=0)
        max_c = np.max(verts, axis=0)
        
        sx, sy, sz = model.scale
        px, py, pz = model.position
        
        model.bbox_min = [min_c[0]*sx + px, min_c[1]*sy + py, min_c[2]*sz + pz]
        model.bbox_max = [max_c[0]*sx + px, max_c[1]*sy + py, max_c[2]*sz + pz]


def euler_forward(pitch, yaw):
    """Euler angles to forward vector"""
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)
    return np.array([
        np.cos(pitch_rad) * np.sin(yaw_rad),
        np.sin(pitch_rad),
        np.cos(pitch_rad) * np.cos(yaw_rad)
    ])