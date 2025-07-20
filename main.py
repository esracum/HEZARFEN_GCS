import sys
from PyQt5.QtWidgets import QApplication
from main_ui import DroneGCS

# PROGRAMI BAŞLAT
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DroneGCS()
    window.show()
    sys.exit(app.exec_())