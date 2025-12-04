#    BİSMİLLAHİRRAHMANİRRAHİM Fİ EVVELİHİ VEL AHİRİ                                                      #
#    BİSMİLLAHİRRAHMANİRRAHİM                      بسم الله الرحمن الرحيم                              #
##########################################################################################################
#    Şems Suresi 7-9                                                                                     #
#    Nefse ve onu düzgün bir biçimde şekillendirip                                                       # 
#    ona kötülük duygusunu ve takvasını (kötülükten sakınma yeteneğini) ilham edene andolsun ki,         #
#    nefsini arındıran kurtuluşa ermiştir.                                                               #
##########################################################################################################
#    ALLAH, insana beyanı verendir.                                                                      # 
#    insan bu beyanla kötülük yapmakla iyilik yapmak arasında seçim yapar.                               #
#    kötü amellerde veya iyi amellerde bulunanlar karşılığını eksiksiz alacaktır.                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
#                                                                                                        #
##########################################################################################################




import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
#from PyQt5.QtGui import QColor
from src.simulation import OpenGLWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle("Simple OpenGL Application")
        self.setGeometry(100, 100, 800, 600)
        self.opengl_widget = OpenGLWidget()
        self.setCentralWidget(self.opengl_widget)

        # Timer for redrawing
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.opengl_widget.update)
        self.timer.start(16)  # Approx 60 FPS

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
