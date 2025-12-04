from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt5.QtWidgets import QOpenGLWidget, QApplication
from PyQt5.QtCore import Qt, QPoint, QTimer
from PyQt5.Qt import QCursor
from PyQt5.QtGui import QImage
import time, io, traceback, contextlib, threading, queue, math, os, json
from PyQt5.QtWidgets import QOpenGLWidget, QMessageBox
from PyQt5.QtCore import QTimer,Qt
from OpenGL.GL import glGetString, GL_VERSION
from src.modules.plane_model import Uav_Model
from src.modules.static_models import StaticModels
from src.utils.math_utils import *
import time,numpy as np
from config.Config import *
CAM_FPS = 0; CAM_FREE = 1; CAM_3RD = 2
CAM_NAMES = {CAM_FPS: "FPS", CAM_FREE: "FREE", CAM_3RD: "3RD"}

try:
    #import pybullet as p
    PHYSICS_AVAILABLE = True
except ImportError:
    print("PyBullet bulunamadı. Fizik simülasyonu devre dışı.")
    PHYSICS_AVAILABLE = False
class OpenGLWidget(QOpenGLWidget):
    def __init__(self):
        super().__init__()
        self.Plane = Uav_Model()
        self._action_queue = queue.Queue(); self._output_queue = queue.Queue()
        self.rotation_angle = 0.0
        self.timer = self.startTimer(16)                                                                      
        self.camera_mode = CAM_FPS; self.keys_down = set()
        self.free_pos = [0.0, 1.5, 0.0]; self.free_rot = [0.0, 0.0, 0.0] #free kamera pozisyonu ve rotasyonu
        self.camera_view_offset = [0.0, 0.0]
        self.mouse_sensitivity = 0.15; self.move_speed = 3.5; self.last_time = time.time()
        self.timer = QTimer(self); self.timer.timeout.connect(self.on_frame); self.timer.start(16) # yaklaşık 60 FPS için 1000ms/60 ≈ 16ms  
        self.mouse_x_inverted = True; self.mouse_y_inverted = False; self.mouse_captured = False

        self.setFocusPolicy(Qt.StrongFocus); self.setMouseTracking(True)

        self.static_models = []  # Statik objeler listesi (zemin, binalar)
        # model data + GL resources
        self.model_vertices = None
        self.model_texcoords = None
        self.model_count = 0
        self.vbo_vertices = None
        self.vbo_texcoords = None
        self.texture_id = None
        self.use_vbo = True

        # Physics variables
        self.physics_enabled = False
        self.physics_client = None
        self.body_id = None
        self.start_time = None
        self.last_velocity = 0.0
        self.last_velocity_time = 0.0
        self.physics_timer = None
        self.rotation_in_radians = False

        self.pitch_limits = (None, None)   # tipik pitch sınırı (dik bakışı engellemek için)
        self.yaw_limits   = (None, None)    # yaw genellikle sınırsız; istersen (-180,180) gibi koy
        self.roll_limits  = (None, None)    # roll için örn: (-45,45)

    def initializeGL(self):
        try:
            glClearColor(0.0, 0.0, 0.0, 1.0)  # Background color: black
            glEnable(GL_MULTISAMPLE);# glHint(GL_MULTISAMPLE, GL_NICEST); # bu kısım anti alisign için
            glEnable(GL_DEPTH_TEST) # Derinlik testine yarar ön plan - arkaplan hiyerarşisini ( z-dept ) yönetmeye yarar.
            glEnable(GL_CULL_FACE) # OpenGL'de yüzlerin görünürlüğünü kontrol etmek için kullanılır ve belirli bir 3D nesnenin hangi taraflarının çizileceğini belirler.
            glClearColor(0.07,0.09,0.15,1.0)  # arkaplan rengi
            self._load_default_models() # modellerin yüklenmesi
        except Exception as e:
            QMessageBox.critical(None, "Initialization Error", str(e))
    def _load_default_models(self):
        """Varsayılan modelleri yükle - GL context hazır olduktan sonra çağrılır"""
        try:
            # Ana drone modeli
            if os.path.isfile(DEFAULT_MODEL_PATH):
                tex = DEFAULT_TEXTURE_PATH if os.path.isfile(DEFAULT_TEXTURE_PATH) else None
                success = self.load_model_with_texture(DEFAULT_MODEL_PATH, tex)
                print(f"Drone model yüklemesi: {'Başarılı' if success else 'Başarısız'}")
            else:
                print(f"HATA: Drone model dosyası bulunamadı: {DEFAULT_MODEL_PATH}")
            
            # Zemin modeli
            if os.path.isfile(DEFAULT_LAXTON_PATH):
                ground_id = self.add_ground_model(
                    obj_path=DEFAULT_LAXTON_PATH, 
                    position=[0, -1.85, 300],
                    rotation=[0, 90, 0],
                    scale=[1, 1, 1]
                )
                success = self.load_static_model(ground_id, DEFAULT_LAXTON_TEXTURE_PATH)
                print(f"Zemin model yüklemesi: {'Başarılı' if success else 'Başarısız'}")
            else:
                print(f"HATA: Zemin model dosyası bulunamadı: {DEFAULT_LAXTON_PATH}")
            
            # Yol modeli
            if os.path.isfile(DEFAULT_ROAD_PATH):
                road_id = self.add_building_model(
                    obj_path=DEFAULT_ROAD_PATH,
                    position=[0, -2, 7],
                    rotation=[0, 90, 0],
                    scale=[1, 1, 1]
                )
                success = self.load_static_model(road_id, DEFAULT_ROAD_TEXTURE_PATH)
                print(f"Yol model yüklemesi: {'Başarılı' if success else 'Başarısız'}")
            else:
                print(f"HATA: Yol model dosyası bulunamadı: {DEFAULT_ROAD_PATH}")
                
            # Cami modeli
            if os.path.isfile(DEFAULT_MOSQUE_PATH):
                building1_id = self.add_building_model(
                    DEFAULT_MOSQUE_PATH,
                    position=[-8, 0.5, 8],
                    rotation=[-10, -20, -3],
                )
                success = self.load_static_model(building1_id, DEFAULT_MOSQUE_TEXTURE_PATH)
                print(f"Cami model yüklemesi: {'Başarılı' if success else 'Başarısız'}")
            else:
                print(f"HATA: Cami model dosyası bulunamadı: {DEFAULT_MOSQUE_PATH}")
                
        except Exception as e:
            print(f"Model yükleme sırasında hata: {e}")
            import traceback
            traceback.print_exc()
    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (width / height), 0.1, 1000.0)
        glMatrixMode(GL_MODELVIEW)

    def on_frame(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        self.rotation_angle += 1.0  # Her zaman 1 derece döner
        if not self.physics_enabled and any(abs(v) > 1e-9 for v in self.Plane.velocity):
            self.Plane.position[0] += self.Plane.velocity[0] * dt
            self.Plane.position[1] += self.Plane.velocity[1] * dt
            self.Plane.position[2] += self.Plane.velocity[2] * dt
        if self.camera_mode == CAM_FREE:
            forward = right = upv = 0.0
            if Qt.Key_W in self.keys_down: forward += 1.0
            if Qt.Key_S in self.keys_down: forward -= 1.0
            if Qt.Key_D in self.keys_down: right -= 1.0
            if Qt.Key_A in self.keys_down: right += 1.0
            if Qt.Key_Space in self.keys_down: upv += 1.0
            if Qt.Key_Control in self.keys_down: upv -= 1.0
            yaw = radians(self.free_rot[1]+180)
            forward_vec = (sin(yaw) * forward, 0.0, cos(yaw) * forward)
            right_vec = (cos(yaw) * right, 0.0, -sin(yaw) * right)
            dx = (forward_vec[0] + right_vec[0]) * self.move_speed * dt
            dy = upv * self.move_speed * dt
            dz = (forward_vec[2] + right_vec[2]) * self.move_speed * dt
            self.free_pos[0] += dx
            self.free_pos[1] += dy
            self.free_pos[2] += dz
        else:
                # Kuvvet hesaplama
                forward = right = upv = 0.0
                if Qt.Key_W in self.keys_down: forward += 1.0
                if Qt.Key_S in self.keys_down: forward -= 1.0
                if Qt.Key_D in self.keys_down: right -= 1.0
                if Qt.Key_A in self.keys_down: right += 1.0
                if Qt.Key_Space in self.keys_down: upv += 1.0
                if Qt.Key_Control in self.keys_down: upv -= 1.0
                
                yaw = radians(self.Plane.rotation[1]+180)
                forward_vec = (sin(yaw) * forward, 0.0, cos(yaw) * forward)
                right_vec = (cos(yaw) * right, 0.0, -sin(yaw) * right)
                
                # Kuvvet vektörlerini hesapla
                force_x = forward_vec[0] + right_vec[0]
                force_z = forward_vec[2] + right_vec[2]
                
                # Fizik parametreleri
                mass = getattr(self.Plane, 'mass', 1.0)  # Kütle
                friction = getattr(self.Plane, 'friction', 0.1)  # Sürtünme katsayısı
                max_force = getattr(self.Plane, 'max_force', 10.0)  # Maksimum kuvvet
                
                # Kuvveti sınırla
                total_horizontal_force = (force_x**2 + force_z**2)**0.5
                if total_horizontal_force > max_force:
                    force_x = force_x * max_force / total_horizontal_force
                    force_z = force_z * max_force / total_horizontal_force
                
                # Yatay hız güncellemesi (her durumda fizik tabanlı)
                if hasattr(self.Plane, 'velocity'):
                    # Sürtünme kuvveti (hızın tersi yönünde)
                    friction_force_x = -self.Plane.velocity[0] * friction
                    friction_force_z = -self.Plane.velocity[2] * friction
                    
                    # Toplam kuvvet = uygulanan kuvvet + sürtünme
                    total_force_x = force_x * self.move_speed + friction_force_x
                    total_force_z = force_z * self.move_speed + friction_force_z
                    
                    # İvme hesaplama (F = ma -> a = F/m)
                    accel_x = total_force_x / mass
                    accel_z = total_force_z / mass
                    
                    # Hız güncellemesi
                    self.Plane.velocity[0] += accel_x * dt
                    self.Plane.velocity[2] += accel_z * dt
                
                # Dikey hareket (fizikten bağımsız)
                dy = upv * self.move_speed * dt
                
                # Pozisyon güncellemesi
                if self.physics_enabled:
                    # Fizik etkinse sadece dikey pozisyonu manuel güncelle
                    # Yatay hareket PyBullet tarafından yönetiliyor
                    if dy != 0:
                        self.Plane.position[1] += dy
                else:
                    # Fizik devre dışıysa tüm hareketi manuel güncelle
                    self.Plane.translate(0, dy, 0)  # Sadece dikey hareket
                    # Yatay hareket velocity ile yapılacak (üstteki kod bloğunda)
        
        self.Plane._save_if_changed() # her framede jsonda değişiklik varsa json güncellenir
        self.update()  # Arayüzü günceller
    def keyPressEvent(self, ev):
        k = ev.key()
        if k == Qt.Key_M:
            self.camera_mode = (self.camera_mode + 1) % 3
            top = self.window()
            if top: top.setWindowTitle(f"Camera: {CAM_NAMES[self.camera_mode]} ")
            return
        if k == Qt.Key_Q:
            self.toggle_mouse_capture(); return
        self.keys_down.add(k)

    def keyReleaseEvent(self, ev):
        if ev.key() in self.keys_down: self.keys_down.remove(ev.key())

    def toggle_mouse_capture(self):
        self.mouse_captured = not self.mouse_captured
        if self.mouse_captured:
            QApplication.setOverrideCursor(Qt.BlankCursor)
            center_local = QPoint(self.width() // 2, self.height() // 2)
            center_global = self.mapToGlobal(center_local)
            QCursor.setPos(center_global)
        else:
            try: QApplication.restoreOverrideCursor()
            except Exception: pass
        top = self.window()
        if top: top.setWindowTitle(f"Camera: {CAM_NAMES[self.camera_mode]} - Manual:  - Capture: {self.mouse_captured}")

    def mouseMoveEvent(self, ev):
        if not self.mouse_captured:
            super().mouseMoveEvent(ev)
            return

        center_local = QPoint(self.width() // 2, self.height() // 2)
        center_global = self.mapToGlobal(center_local)

        dx = ev.x() - center_local.x()
        dy = ev.y() - center_local.y()

        sign_x = -1.0 if getattr(self, "mouse_x_inverted", False) else 1.0
        sign_y = -1.0 if getattr(self, "mouse_y_inverted", False) else 1.0

        yaw_delta = sign_x * dx * self.mouse_sensitivity
        pitch_delta = sign_y * dy * self.mouse_sensitivity

        max_step = getattr(self, "mouse_max_step", 8.0)
        yaw_delta = max(-max_step, min(max_step, yaw_delta))
        pitch_delta = max(-max_step, min(max_step, pitch_delta))

        # Opsiyonel: Shift ile roll kontrolü (örnek)
        if ev.modifiers() & Qt.ShiftModifier:
            # yatay hareket roll olarak kullanılacak
            self.free_rot[2] += yaw_delta
        else:
            self.free_rot[1] += yaw_delta
            self.free_rot[0] += pitch_delta

        # Sınırlandırmalar (varsa uygula)
        pmin, pmax = getattr(self, "pitch_limits", (None, None))
        ymin, ymax = getattr(self, "yaw_limits", (None, None))
        rmin, rmax = getattr(self, "roll_limits", (None, None))

        # Pitch clamp (dikey bakış için kesinlikle uygula genelde)
        self.free_rot[0] = clamp(self.free_rot[0], pmin, pmax)

        # Yaw/roll: eğer sınır verilmişse clamp et; değilse normalize et (kararlılık için)
        if (ymin is not None) or (ymax is not None):
            self.free_rot[1] = clamp(self.free_rot[1], ymin, ymax)
        else:
            self.free_rot[1] = normalize_angle_deg(self.free_rot[1])

        if (rmin is not None) or (rmax is not None):
            self.free_rot[2] = clamp(self.free_rot[2], rmin, rmax)
        else:
            self.free_rot[2] = normalize_angle_deg(self.free_rot[2])

        # Uygula: Plane.rotation hangi birimi bekliyorsa ona göre ver
        rot_to_set = list(self.free_rot)
        if getattr(self, "rotation_in_radians", False):
            rot_to_set = [math.radians(x) for x in rot_to_set]

        if self.camera_mode == CAM_FREE:
            # Kamera kontrol katmanına göre atama yap
            self.free_rot = rot_to_set if not self.rotation_in_radians else [math.degrees(x) for x in rot_to_set]
            self.Plane.rotation = rot_to_set
        else:
            self.Plane.rotation = rot_to_set

        QCursor.setPos(center_global)
        super().mouseMoveEvent(ev)

    def mousePressEvent(self, ev): super().mousePressEvent(ev)
    def mouseReleaseEvent(self, ev): super().mouseReleaseEvent(ev)


    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        eye, target, up = self._compute_camera_view()
        gluLookAt(eye[0],eye[1],eye[2], target[0],target[1],target[2], up[0],up[1],up[2])

        self._draw_grid()
        # STATİK MODELLERİ ÇİZ (zemin, binalar)
        for model in self.static_models:
            if model.loaded:
                glPushMatrix()
                glTranslatef(*model.position)
                glRotatef(model.rotation[0], 1, 0, 0)
                glRotatef(model.rotation[1], 0, 1, 0) 
                glRotatef(model.rotation[2], 0, 0, 1)
                glScalef(*model.scale)
                self._draw_static_model(model)
                glPopMatrix()
        # DRONE MODELİNİ ÇİZ
        if self.model_vertices is not None and self.model_count > 0:
            glPushMatrix()
            glTranslatef(*self.Plane.position)
            glRotatef(self.Plane.rotation[0],1,0,0)
            glRotatef(self.Plane.rotation[1],0,1,0)
            glRotatef(self.Plane.rotation[2],0,0,1)
            glScalef(*self.Plane.scale)
            self._draw_model_with_vertex_array()
            glPopMatrix()
        else:
            glPushMatrix()
            glTranslatef(*self.Plane.position)
            glRotatef(self.Plane.rotation[0],1,0,0); glRotatef(self.Plane.rotation[1],0,1,0); glRotatef(self.Plane.rotation[2],0,0,1)
            glScalef(*self.Plane.scale)
            self._draw_colored_cube(self.Plane.color)
            glPopMatrix()
    def _draw_colored_cube(self, color):
        r,g,b = color
        glBegin(GL_QUADS)
        glColor3f(r,g,b); glVertex3f(-0.5,-0.5,0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(-0.5,0.5,0.5)
        glColor3f(r*0.9,g*0.9,b*0.9); glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,0.5,-0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,-0.5,-0.5)
        glColor3f(r*0.8,g*0.8,b*0.8); glVertex3f(-0.5,-0.5,-0.5); glVertex3f(-0.5,-0.5,0.5); glVertex3f(-0.5,0.5,0.5); glVertex3f(-0.5,0.5,-0.5)
        glColor3f(r,g,b); glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,0.5,-0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,-0.5,0.5)
        glColor3f(r*0.95,g*0.95,b*0.95); glVertex3f(-0.5,0.5,-0.5); glVertex3f(-0.5,0.5,0.5); glVertex3f(0.5,0.5,0.5); glVertex3f(0.5,0.5,-0.5)
        glColor3f(r*0.7,g*0.7,b*0.7); glVertex3f(-0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,-0.5); glVertex3f(0.5,-0.5,0.5); glVertex3f(-0.5,-0.5,0.5)
        glEnd()
    def _draw_model_with_vertex_array(self):
        if self.use_vbo and self.vbo_vertices:
            if self.texture_id is not None:
                glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, int(self.texture_id))
            if self.vbo_texcoords:
                glEnableClientState(GL_TEXTURE_COORD_ARRAY)
                glBindBuffer(GL_ARRAY_BUFFER, int(self.vbo_texcoords))
                glTexCoordPointer(2, GL_FLOAT, 0, None)
            glEnableClientState(GL_VERTEX_ARRAY)
            glBindBuffer(GL_ARRAY_BUFFER, int(self.vbo_vertices))
            glVertexPointer(3, GL_FLOAT, 0, None)
            r,g,b = self.Plane.color; glColor3f(r,g,b)
            glDrawArrays(GL_TRIANGLES, 0, self.model_count)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            glDisableClientState(GL_VERTEX_ARRAY)
            if self.vbo_texcoords:
                glDisableClientState(GL_TEXTURE_COORD_ARRAY)
            if self.texture_id is not None:
                glBindTexture(GL_TEXTURE_2D, 0); glDisable(GL_TEXTURE_2D)
            return

        # fallback: client arrays
        if self.model_vertices is None: return
        ptr = self.model_vertices.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        if self.model_texcoords is not None:
            texptr = self.model_texcoords.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
            glEnableClientState(GL_TEXTURE_COORD_ARRAY); glTexCoordPointer(2, GL_FLOAT, 0, texptr)
        if self.texture_id is not None:
            glEnable(GL_TEXTURE_2D); glBindTexture(GL_TEXTURE_2D, int(self.texture_id))
        r,g,b = self.cube.color; glColor3f(r,g,b)
        glEnableClientState(GL_VERTEX_ARRAY); glVertexPointer(3, GL_FLOAT, 0, ptr)
        glDrawArrays(GL_TRIANGLES, 0, self.model_count)
        glDisableClientState(GL_VERTEX_ARRAY)
        if self.model_texcoords is not None:
            glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        if self.texture_id is not None:
            glBindTexture(GL_TEXTURE_2D, 0); glDisable(GL_TEXTURE_2D)


    def _draw_static_model(self, model: StaticModels):
        """Statik model çizimi"""
        if model.vbo_vertices:
            glEnableClientState(GL_VERTEX_ARRAY)
            glBindBuffer(GL_ARRAY_BUFFER, model.vbo_vertices)
            glVertexPointer(3, GL_FLOAT, 0, None)
            
            if model.vbo_texcoords and model.texture_id:
                glEnable(GL_TEXTURE_2D)
                glBindTexture(GL_TEXTURE_2D, model.texture_id)
                glEnableClientState(GL_TEXTURE_COORD_ARRAY)
                glBindBuffer(GL_ARRAY_BUFFER, model.vbo_texcoords)
                glTexCoordPointer(2, GL_FLOAT, 0, None)
            
            glColor3f(*model.color)
            glDrawArrays(GL_TRIANGLES, 0, model.vertex_count)
            
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            glDisableClientState(GL_VERTEX_ARRAY)
            if model.vbo_texcoords:
                glDisableClientState(GL_TEXTURE_COORD_ARRAY)
                glDisable(GL_TEXTURE_2D)
    
    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        glColor3f(0.2,0.2,0.2)
        glBegin(GL_LINES)
        for i in range(-10,11):
            glVertex3f(i,-1.0,-10); glVertex3f(i,-1.0,10)
            glVertex3f(-10,-1.0,i); glVertex3f(10,-1.0,i)
        glEnd()
    def _compute_camera_view(self):
        up = (0.0, 1.0, 0.0)

        if self.camera_mode == CAM_FPS:
            eye_offset = (0.0, 0.2, 0.0)
            eye = (
                self.Plane.position[0] + eye_offset[0],
                self.Plane.position[1] + eye_offset[1],
                self.Plane.position[2] + eye_offset[2]
            )
            pitch = self.Plane.rotation[0]
            yaw = self.Plane.rotation[1] + 180.0  # Y eksenine göre 180° ters çevir
            forward = euler_forward(pitch, yaw)
            target = (eye[0] + forward[0], eye[1] + forward[1], eye[2] + forward[2])
            return eye, target, up

        if self.camera_mode == CAM_FREE:
            eye = tuple(self.free_pos)
            pitch = self.free_rot[0]
            yaw = self.free_rot[1] + 180.0  # Y eksenine göre 180° ters çevir
            forward = euler_forward(pitch, yaw)
            target = (eye[0] + forward[0], eye[1] + forward[1], eye[2] + forward[2])
            return eye, target, up

        # === FOLLOW CAMERA ===
        distance = 4.0
        height = 1.5
        yaw = radians(self.Plane.rotation[1] + 180.0)  # Y eksenine göre 180° çevir
        offset_x = -sin(yaw) * distance
        offset_z = -cos(yaw) * distance
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

    
    def add_ground_model(self, obj_path: str, position: list = None,rotation:list=None, scale: list = None, texture_path: str = None):
        """Zemin modeli ekle"""
        ground = StaticModels(obj_path, position or [0, -2, 0],rotation or [0, 0, 0], scale or [10, 1, 10])
        ground.color = (1, 1, 1)  # Beyaz zemin
        self.static_models.append(ground)
        return len(self.static_models) - 1  # Model ID döndür
    def load_model_with_texture(self, obj_path: str, texture_path: str=None, apply_default_rotation=True, use_vbo=True):
        vert_flat, uv_flat = self.parse_obj_file(obj_path)
        if vert_flat is None:
            return False

        self.use_vbo = bool(use_vbo)
        if self.use_vbo:
            try:
                self._create_vbos(vert_flat, uv_flat)
            except Exception as e:
                self._output_queue.put((False, f"VBO creation failed: {e}"))
                self.model_vertices = np.array(vert_flat, dtype=np.float32)
                self.model_texcoords = np.array(uv_flat, dtype=np.float32) if uv_flat else None
                self.model_count = int(len(vert_flat) // 3)
                self.vbo_vertices = None; self.vbo_texcoords = None
                self.use_vbo = False
        else:
            self.model_vertices = np.array(vert_flat, dtype=np.float32)
            self.model_texcoords = np.array(uv_flat, dtype=np.float32) if uv_flat else None
            self.model_count = int(len(vert_flat) // 3)

        if apply_default_rotation:
            try:
                self.cube.set_rotation(*DEFAULT_MODEL_ROTATION)
            except Exception:
                pass

        tex_loaded = False
        if texture_path and os.path.isfile(texture_path):
            tex_loaded = self.load_texture(texture_path)
        else:
            base = os.path.splitext(obj_path)[0]
            for ext in ('.png', '.jpg', '.jpeg', '.tga'):
                texp = base + ext
                if os.path.isfile(texp):
                    tex_loaded = self.load_texture(texp)
                    if tex_loaded: break

        self._output_queue.put((True, f"Model loaded (with_texture={bool(tex_loaded)}): {obj_path}"))
        return True
    def parse_obj_file(self, path):
        verts = []
        uvs = []
        faces = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'): continue
                    parts = line.split()
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
                            if vi < 0: vi = len(verts) + 1 + vi
                            vi -= 1
                            idxs.append(vi)
                            if len(comps) >= 2 and comps[1] != '':
                                vti = int(comps[1])
                                if vti < 0: vti = len(uvs) + 1 + vti
                                uv_idxs.append(vti - 1)
                            else:
                                uv_idxs.append(None)
                        for i in range(1, len(idxs)-1):
                            faces.append(((idxs[0], uv_idxs[0]), (idxs[i], uv_idxs[i]), (idxs[i+1], uv_idxs[i+1])))
        except Exception as e:
            self._output_queue.put((False, f"OBJ parse error: {e}"))
            return None, None

        vert_flat = []
        uv_flat = []
        for tri in faces:
            for vi, vti in tri:
                try:
                    x,y,z = verts[vi]
                except Exception as e:
                    self._output_queue.put((False, f"OBJ index error: {e}"))
                    return None, None
                vert_flat.extend([x,y,z])
                if vti is not None and 0 <= vti < len(uvs):
                    u,v = uvs[vti]
                    uv_flat.extend([u, v])
                else:
                    uv_flat.extend([0.0, 0.0])

        if len(vert_flat) == 0:
            self._output_queue.put((False, "OBJ: no triangles found"))
            return None, None

        return vert_flat, uv_flat
    def _create_vbos(self, vert_array, uv_array=None):
        vbuf = np.array(vert_array, dtype=np.float32)
        self.model_vertices = vbuf
        self.model_count = int(len(vert_array) // 3)

        try:
            if self.vbo_vertices:
                glDeleteBuffers(1, [int(self.vbo_vertices)])
                self.vbo_vertices = None
            if self.vbo_texcoords:
                glDeleteBuffers(1, [int(self.vbo_texcoords)])
                self.vbo_texcoords = None
        except Exception:
            pass

        self.vbo_vertices = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, int(self.vbo_vertices))
        glBufferData(GL_ARRAY_BUFFER, vbuf.nbytes, vbuf, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        if uv_array is not None and len(uv_array) >= 2:
            tbuf = np.array(uv_array, dtype=np.float32)
            self.model_texcoords = tbuf
            self.vbo_texcoords = glGenBuffers(1)
            glBindBuffer(GL_ARRAY_BUFFER, int(self.vbo_texcoords))
            glBufferData(GL_ARRAY_BUFFER, tbuf.nbytes, tbuf, GL_STATIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
        else:
            self.model_texcoords = None
            self.vbo_texcoords = None
    def load_texture(self, image_path: str) -> bool:
        try:
            img = QImage(image_path)
            if img.isNull():
                self._output_queue.put((False, f"Texture load failed: cannot open {image_path}"))
                return False
            img = img.convertToFormat(QImage.Format_RGBA8888)
            img = img.mirrored(False, True)
            w = img.width(); h = img.height()
            ptr = img.bits(); ptr.setsize(img.byteCount()); data = bytes(ptr)

            if self.texture_id is None:
                self.texture_id = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, int(self.texture_id))
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
            try: glGenerateMipmap(GL_TEXTURE_2D)
            except Exception: pass
            glBindTexture(GL_TEXTURE_2D, 0)
            self._output_queue.put((True, f"Texture loaded: {image_path} ({w}x{h})"))
            return True
        except Exception as e:
            self._output_queue.put((False, f"Texture load error: {e}"))
            return False
    def add_building_model(self, obj_path: str, position: list, rotation: list = None, scale: list = None):
        """Bina/engel modeli ekle"""
        building = StaticModels(obj_path, position, rotation or [0, 0, 0], scale or [1, 1, 1])
        building.color = (0.6, 0.5, 0.4)  # Kahverengi bina
        self.static_models.append(building)
        return len(self.static_models) - 1

    def load_static_model(self, model_id: int, texture_path: str = None):
        """Statik modeli yükle (GL context'te çağır)"""
        if model_id >= len(self.static_models):
            return False

        model = self.static_models[model_id]
        vert_flat, uv_flat = self.parse_obj_file(model.obj_path)

        if vert_flat is None:
            return False

        # Model verilerini kaydet
        model.vertices = np.array(vert_flat, dtype=np.float32)
        model.vertex_count = len(vert_flat) // 3

        # UV verisi varsa kontrol et (karmaşık .obj'lerde indeks/face uyuşmazlığı olabilir)
        if uv_flat:
            uv_arr = np.array(uv_flat, dtype=np.float32)
            # Beklenen UV sayısı vertex_count * 2 (u,v)
            if uv_arr.size != model.vertex_count * 2:
                # Uyumsuz ise uvs'i yok say (ve logla)
                self._output_queue.put((False, f"UV count mismatch: expected {model.vertex_count*2}, got {uv_arr.size}. Ignoring texcoords for {model.obj_path}"))
                model.texcoords = None
            else:
                model.texcoords = uv_arr
        else:
            model.texcoords = None

        # VBO oluştur
        try:
            # Vertex VBO
            vbo_v = glGenBuffers(1)
            # glGenBuffers bazen bir dizi dönebilir, int'e çevir
            try:
                vbo_v = int(vbo_v)
            except Exception:
                pass
            model.vbo_vertices = vbo_v
            glBindBuffer(GL_ARRAY_BUFFER, model.vbo_vertices)
            glBufferData(GL_ARRAY_BUFFER, model.vertices.nbytes, model.vertices, GL_STATIC_DRAW)

            # Texcoord VBO (varsa)
            if model.texcoords is not None:
                vbo_t = glGenBuffers(1)
                try:
                    vbo_t = int(vbo_t)
                except Exception:
                    pass
                model.vbo_texcoords = vbo_t
                glBindBuffer(GL_ARRAY_BUFFER, model.vbo_texcoords)
                glBufferData(GL_ARRAY_BUFFER, model.texcoords.nbytes, model.texcoords, GL_STATIC_DRAW)

            # Temizle
            glBindBuffer(GL_ARRAY_BUFFER, 0)

        except Exception as e:
            self._output_queue.put((False, f"Static model VBO error: {e}"))
            return False

        # Texture yükle (varsa)
        tex_loaded = False
        if texture_path and os.path.isfile(texture_path):
            tex_loaded = self.load_texture_m(texture_path, assign_to=model)
        else:
            base = os.path.splitext(model.obj_path)[0]
            for ext in ('.png', '.jpg', '.jpeg', '.tga'):
                texp = base + ext
                if os.path.isfile(texp):
                    tex_loaded = self.load_texture_m(texp, assign_to=model)
                    if tex_loaded:
                        break

        # Çarpışma kutusunu hesapla
        self.calculate_bounding_box(model)
        model.loaded = True

        self._output_queue.put((True, f"Static model loaded (with_texture={bool(tex_loaded)}): {model.obj_path}"))
        return True


    def load_texture_m(self, texture_path: str, assign_to=None):
        try:
            # Local importlar (fonksiyon tek başına çağrılsa da çalışsın)
            from OpenGL.GL import (
                glGenTextures, glBindTexture, glTexImage2D, glGenerateMipmap,
                glTexParameteri, glPixelStorei, GL_UNPACK_ALIGNMENT,
                GL_TEXTURE_2D, GL_RGBA, GL_UNSIGNED_BYTE,
                GL_TEXTURE_MIN_FILTER, GL_TEXTURE_MAG_FILTER,
                GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR,
                GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_REPEAT
            )

            # OpenGL texture oluştur
            tex_id = glGenTextures(1)
            # Bazı PyOpenGL sürümlerinde liste/tuple dönebilir -> int'e çevir
            try:
                tex_id = int(tex_id)
            except Exception:
                # eğer ndarray veya benzeri ise ilk elemanı al
                try:
                    tex_id = int(tex_id[0])
                except Exception:
                    pass

            glBindTexture(GL_TEXTURE_2D, tex_id)

            # Byte hizalamasını ayarla (tightly packed veriler için 1 güvenli seçimdir)
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1)

            # QImage ile resmi oku
            img = QImage(texture_path)
            if img.isNull():
                raise ValueError(f"Failed to load image: {texture_path}")

            # ConvertToFormat uygunluk kontrolü: modern Qt'de Format_RGBA8888 bulunur,
            # yoksa Format_ARGB32 fallback kullanıp kanalları yeniden düzenleyeceğiz.
            need_channel_swap = False
            try:
                # Tercihen RGBA8888 al, böylece veri doğrudan RGBA olur
                img = img.convertToFormat(QImage.Format_RGBA8888)
                need_channel_swap = False
            except Exception:
                # fallback: ARGB32 (genelde little-endian'da BGRA olarak gelir)
                img = img.convertToFormat(QImage.Format_ARGB32)
                need_channel_swap = True

            width = img.width()
            height = img.height()

            # Byte verisini numpy array'e al
            ptr = img.bits()
            ptr.setsize(img.byteCount())
            img_data = np.array(ptr, dtype=np.uint8).reshape((height, width, 4))

            # Eğer fallback kullanıldıysa (ARGB32 -> BGRA gibi gelir), kanalları RGBA'ya döndür
            if need_channel_swap:
                # BGRA -> RGBA : (B,G,R,A) -> (R,G,B,A) için indeksleme
                img_data = img_data[..., [2, 1, 0, 3]]

            # OpenGL tipik olarak (0,0) sol-altı bekler; QImage (0,0) sol-üst olabilir
            # Bu nedenle dikey olarak çeviriyoruz (çoğu .obj pipeline'ında gereklidir)
            img_data = np.flip(img_data, axis=0)

            # Bellek düzenini garanti et (contiguous)
            img_data = np.ascontiguousarray(img_data, dtype=np.uint8)

            # OpenGL'e yükle (RGBA, UNSIGNED_BYTE)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                        GL_RGBA, GL_UNSIGNED_BYTE, img_data)

            # Mipmaps ve parametreler
            glGenerateMipmap(GL_TEXTURE_2D)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)

            # Temizle
            glBindTexture(GL_TEXTURE_2D, 0)

            # Eğer bir model verilmişse ona texture id ata
            if assign_to is not None:
                # güvenli int atama
                try:
                    assign_to.texture_id = int(tex_id)
                except Exception:
                    assign_to.texture_id = tex_id
            else:
                try:
                    self.texture_id = int(tex_id)
                except Exception:
                    self.texture_id = tex_id

            return True
        except Exception as e:
            self._output_queue.put((False, f"Texture load error: {e}"))
            return False
    def calculate_bounding_box(self, model: StaticModels):
        """Model için çarpışma kutusunu hesapla"""
        if model.vertices is None or len(model.vertices) < 3:
            return
            
        # Vertex verilerini 3'erli gruplara ayır
        vertices = model.vertices.reshape(-1, 3)
        
        # Min/Max koordinatları bul
        min_coords = np.min(vertices, axis=0)
        max_coords = np.max(vertices, axis=0)
        
        # Scale ve position uygula
        sx, sy, sz = model.scale
        px, py, pz = model.position
        
        model.bbox_min = [
            min_coords[0] * sx + px,
            min_coords[1] * sy + py, 
            min_coords[2] * sz + pz
        ]
        model.bbox_max = [
            max_coords[0] * sx + px,
            max_coords[1] * sy + py,
            max_coords[2] * sz + pz
        ]
        
        print(f"Model bbox: min={model.bbox_min}, max={model.bbox_max}")
    # ===== ÇARPIŞMA KONTROLÜ =====
    
