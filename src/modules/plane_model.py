from typing import Sequence, List, Any
import os,json,tempfile

class Uav_Model:
    def __init__(self, json_path: str = "DataShared/drone_state.json"):
        self.json_path = json_path

        self.time: float = 0.0
        self.position: List[float]  = [0.0, 0.0, -7.0]
        self.rotation: List[float]  = [0.0, 0.0, 0.0]  # pitch (x), yaw (y), roll (z) 
        self.scale: List[float]  = [1.0, 1.0, 1.0]
        self.color = (1.0, 1.0, 1.0) # model rengi rgb
        self.velocity: List[float]  = [0.0, 0.0, 0.0]
        self.acceleration: float = 0.0
        # son kaydedilen durum (karşılaştırma için)
        self._last_saved_state: Any = None

    def set_position(self, x, y, z): self.position = [float(x), float(y), float(z)]
    def translate(self, dx, dy, dz):
        self.position[0] += float(dx); self.position[1] += float(dy); self.position[2] += float(dz)
    def set_rotation(self, rx, ry, rz): self.rotation = [float(rx), float(ry), float(rz)]
    def rotate(self, drx, dry, drz): self.rotation[0] += float(drx); self.rotation[1] += float(dry); self.rotation[2] += float(drz)
    def set_scale(self, sx, sy, sz): self.scale = [float(sx), float(sy), float(sz)]
    def set_color(self, r, g, b): self.color = (float(r), float(g), float(b))
    def set_velocity(self, vx, vy, vz): self.velocity = [float(vx), float(vy), float(vz)]
    def apply_impulse(self, ix, iy, iz): self.velocity[0]+=float(ix); self.velocity[1]+=float(iy); self.velocity[2]+=float(iz)
    def set_acceleration(self, acc): self.acceleration = float(acc)
    def set_time(self, t): self.time = float(t)

    # public save (manuel çağrı gerekirse)
    def save_drone_state_to_json(self) -> bool:
        """Mevcut sınıf durumunu JSON'a kaydeder (yalnızca değiştiyse)."""
        return self._save_if_changed()

        # yardımcı: güncel durum sözlüğü
    def _current_state(self) -> dict:
        return {
            "position": list(self.position),
            "orientation": list(self.rotation),
            "velocity": list(self.velocity),
            "acceleration": self.acceleration,
            "time": self.time
        }

    # sadece farklıysa kaydeder
    def _save_if_changed(self) -> bool:
        state = self._current_state()
        if state == self._last_saved_state:
            return False  # değişiklik yok, kaydetme

        # dizin oluştur
        data_dir = os.path.dirname(self.json_path)
        if data_dir and not os.path.exists(data_dir):
            os.makedirs(data_dir, exist_ok=True)

        # atomik yazma: temp -> replace
        try:
            fd, tmp_path = tempfile.mkstemp(dir=data_dir or ".", prefix="._tmp_drone_", text=True)
            with os.fdopen(fd, "w") as f:
                json.dump(state, f, indent=2)
                f.flush()
                os.fsync(f.fileno())
            # atomic replace (platform independent)
            os.replace(tmp_path, self.json_path)
            self._last_saved_state = state
            return True
        except Exception as e:
            # hata olsa bile uygulamayı kırmak istemezsin, sadece bildir
            print(f"JSON kaydetme hatası: {e}")
            # tmp dosya kalırsa temizle
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass
            return False


