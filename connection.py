from PyQt5.QtCore import QThread, pyqtSignal
from dronekit import connect
import time

class DroneConnectionThread(QThread):
    connection_status = pyqtSignal(bool, object)  # success, master

    def __init__(self, connection_string, baudrate, parent=None):
        super().__init__(parent)
        self.connection_string = connection_string
        self.baudrate = baudrate

    def run(self):
        try:
            print(f"🔌 Bağlanıyor: {self.connection_string} @ {self.baudrate}")
            
            # UDP bağlantısı için özel ayarlar
            if self.connection_string.startswith("udp:"):
                # UDP için timeout ve retry ayarları
                master = connect(self.connection_string, wait_ready=False, timeout=10)
                # Bağlantıyı test et
                master.wait_ready(timeout=15)
            else:
                # Serial bağlantı için normal ayarlar
                master = connect(self.connection_string, baud=self.baudrate, wait_ready=True)
            
            print(f"✅ Bağlantı başarılı!")
            self.connection_status.emit(True, master)
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            self.connection_status.emit(False, None)