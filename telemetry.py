from PyQt5.QtCore import QThread, pyqtSignal
import time
import math

class TelemetryThread(QThread):
    telemetry_signal = pyqtSignal(dict)

    def __init__(self, master):
        super().__init__()
        self.master = master
        self.running = True

    def run(self):
        print(f"🔍 Telemetri thread başladı. Master objesi: {type(self.master)}")
        
        while self.running:
            try:
                # GPS pozisyonu ve yükseklik (AGL)
                if hasattr(self.master, 'location') and self.master.location:
                    lat = self.master.location.global_frame.lat
                    lon = self.master.location.global_frame.lon
                    alt = self.master.location.global_relative_frame.alt  # Yerden yükseklik (AGL)
                    
                    print(f"📍 GPS: Lat={lat}, Lon={lon}, Alt(AGL)={alt}")
                    
                    telemetry_data = {
                        'lat': lat,
                        'lon': lon,
                        'alt': alt,
                        'speed': 0,
                        'battery': 0,
                        'yaw': 0,
                        'heading': 0
                    }
                    
                    # Hız bilgisi - groundspeed kullan
                    if hasattr(self.master, 'groundspeed'):
                        telemetry_data['speed'] = self.master.groundspeed
                        print(f"💨 Groundspeed: {self.master.groundspeed}")
                    
                    # Batarya bilgisi
                    if hasattr(self.master, 'battery') and self.master.battery:
                        telemetry_data['battery'] = self.master.battery.level
                        print(f"🔋 Battery Level: {self.master.battery.level}")
                    elif hasattr(self.master, '_voltage'):
                        telemetry_data['battery'] = self.master._voltage
                        print(f"🔋 Voltage: {self.master._voltage}")
                    
                    # Heading bilgisi (daha güvenilir)
                    if hasattr(self.master, 'heading'):
                        telemetry_data['heading'] = self.master.heading
                        print(f"🧭 Heading: {self.master.heading}°")
                    
                    # Yaw açısı (attitude'dan)
                    if hasattr(self.master, 'attitude') and self.master.attitude:
                        yaw_rad = self.master.attitude.yaw
                        yaw_deg = math.degrees(yaw_rad)
                        if yaw_deg < 0:
                            yaw_deg += 360
                        telemetry_data['yaw'] = yaw_deg
                        print(f"🧭 Yaw: {yaw_deg}° (rad: {yaw_rad})")
                    
                    self.telemetry_signal.emit(telemetry_data)
                    print(f"📡 Telemetri verisi gönderildi: {telemetry_data}")
                
            except Exception as e:
                print(f"❌ Telemetri hatası: {e}")
                import traceback
                traceback.print_exc()
            
            self.msleep(2000)  # 2 saniye bekle

    def stop(self):
        self.running = False
        self.quit()
        self.wait()