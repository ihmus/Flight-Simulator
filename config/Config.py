import os 
USERNAME=""
DEFAULT_MODEL_PATH          =   os.path.join(os.getcwd(), "assets/models", "drone.obj")
DEFAULT_TEXTURE_PATH        =   os.path.join(os.getcwd(),"assets/textures","metalicdrone.jpg")
DEFAULT_ROAD_PATH           =   os.path.join(os.getcwd(),"assets/models","road.obj")
DEFAULT_ROAD_TEXTURE_PATH   =   os.path.join(os.getcwd(),"assets/textures","road.jpg")
DEFAULT_MOSQUE_PATH           =   os.path.join(os.getcwd(),"assets/models","mosque.obj")
DEFAULT_MOSQUE_TEXTURE_PATH   =   os.path.join(os.getcwd(),"assets/textures","mosque.jpg")
DEFAULT_LAXTON_PATH           =   os.path.join(os.getcwd(),"assets/models","laxton.obj")
DEFAULT_LAXTON_TEXTURE_PATH   =   os.path.join(os.getcwd(),"assets/textures","laxton.png")

DEFAULT_MODEL_ROTATION = (0.0, 180.0, 0.0)
VERSION = "1.10.24"
IMG_PATH= os.path.join(os.getcwd(), 'img\\app').replace('\\', '/')#Dosya yolunu QPixmap veya QImage  dosya yolunda ters eğik çizgi (\) yerine eğik çizgi (/) kullanmak daha güvenli olabilir.
DB_PATH= os.path.join(os.getcwd(), 'DataBase').replace('\\', '/')
JSON_PATH = os.getcwd().replace("\\", "/") + "/DataShared/drone_state.json"