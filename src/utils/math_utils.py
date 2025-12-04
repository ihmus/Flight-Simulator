import math
from math import sin,cos,radians



def euler_forward(pitch_deg, yaw_deg):
    """
    Docstring for euler_forward
    
    Dönme açılarına göre bir yön vektörü hesaplar.

    Bu fonksiyon, verilen pitch ve yaw açılarını dereceden radyana
    dönüştürerek bir 3D uzayda ileri yön vektörü (x, y, z) döndürür.

    Args:
        pitch_deg (float): Yukarı ve aşağı hareketi temsil eden açıdır.
        yaw_deg (float): Sağ ve sola hareketi temsil eden açıdır.

    Returns:
        tuple: (x, y, z) koordinatları içeren yön vektörü. 
               x ve z, yatay düzlemdeki yönü, y ise dikey yönü belirtir.

    Raises:
        ValueError: Eğer verilen açılar geçersiz bir değer alıyorsa.

    Örnek:
        >>> euler_forward(30, 45)
        (0.6123724356957945, -0.49999999999999994, 0.6123724356957945)
    """
    pitch = math.radians(pitch_deg); yaw = math.radians(yaw_deg)
    x = math.sin(yaw) * math.cos(pitch); y = -math.sin(pitch); z = math.cos(yaw) * math.cos(pitch)
    return (x, y, z)
def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

def euler_to_quat(pitch, yaw, roll):
    # input: derece
    p = deg2rad(pitch) / 2.0
    y = deg2rad(yaw) / 2.0
    r = deg2rad(roll) / 2.0
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)
    sr, cr = math.sin(r), math.cos(r)
    # Tüm rota sıralamalarına dikkat et: burada yaw (Y), pitch (X), roll (Z) sırası alındı
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return (qw, qx, qy, qz)

def quat_to_matrix(q):
    # dönüşüm 3x3 matris döndürür
    qw, qx, qy, qz = q
    # normalizasyon güvenlik
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    # rotasyon matrisi (row-major)
    m00 = 1 - 2*qy*qy - 2*qz*qz
    m01 = 2*qx*qy - 2*qz*qw
    m02 = 2*qx*qz + 2*qy*qw
    m10 = 2*qx*qy + 2*qz*qw
    m11 = 1 - 2*qx*qx - 2*qz*qz
    m12 = 2*qy*qz - 2*qx*qw
    m20 = 2*qx*qz - 2*qy*qw
    m21 = 2*qy*qz + 2*qx*qw
    m22 = 1 - 2*qx*qx - 2*qy*qy
    return [[m00,m01,m02],[m10,m11,m12],[m20,m21,m22]]

def clamp(val, lo, hi):
    if lo is not None and val < lo: return lo
    if hi is not None and val > hi: return hi
    return val

def normalize_angle_deg(a):
    # -180..180 aralığına getirir
    a = (a + 180.0) % 360.0 - 180.0
    return a