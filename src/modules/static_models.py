class StaticModels:
    """Zemin, bina gibi statik objeler için"""
    def __init__(self, obj_path: str, position: list = None, rotation: list = None, scale: list = None):
        self.obj_path = obj_path
        self.position = position or [0.0, 0.0, 0.0]
        self.rotation = rotation or [0.0, 0.0, 0.0] 
        self.scale = scale or [1.0, 1.0, 1.0]
        self.color = (0.8, 0.8, 0.8)  # Varsayılan gri
        
        # Model verileri
        self.vertices = None
        self.texcoords = None 
        self.vertex_count = 0
        self.vbo_vertices = None
        self.vbo_texcoords = None
        self.texture_id = None
        self.loaded = False
        
        # Çarpışma kutusu (AABB - Axis Aligned Bounding Box)
        self.bbox_min = [0, 0, 0]  # Minimum koordinatlar
        self.bbox_max = [0, 0, 0]  # Maksimum koordinatlar
