import time
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame, QGroupBox, QApplication, QComboBox,
    QListWidget, QListWidgetItem, QInputDialog, QScrollArea, QWidget, QLabel
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtCore import QUrl, QObject, pyqtSlot, Qt, QTimer
from PyQt5.QtGui import QFont
from connection import DroneConnectionThread
from telemetry import TelemetryThread
from dronekit import VehicleMode, Command
from pymavlink import mavutil
from flight_modes import (
    set_manual_mode, set_fbwa_mode, set_fbwb_mode, set_cruise_mode, set_stabilize_mode, set_autotune_mode, set_training_mode, set_acro_mode, set_qmodes_mode, set_loiter_mode, set_circle_mode, set_guided_mode, set_rtl_mode, set_takeoff_mode, set_thermal_mode, set_auto_mode, set_auto_mode_print, start_infinity_mission,
    set_acro_copter_mode, set_alt_hold_mode, set_auto_copter_mode, set_autotune_copter_mode, set_brake_mode, set_circle_copter_mode, set_drift_mode, set_flip_mode, set_flowhold_mode, set_follow_mode, set_guided_copter_mode, set_heli_autorotate_mode, set_land_mode, set_loiter_copter_mode, set_poshold_mode, set_rtl_copter_mode, set_simple_mode, set_smartrtl_mode, set_sport_mode, set_stabilize_copter_mode, set_sysid_mode, set_throw_mode, set_turtle_mode, set_zigzag_mode
)
import math
from geo_utils import haversine, generate_infinity_waypoints
from mission_planner import send_waypoints_to_drone
from map_handlers import MapClickHandler

class MarqueeLabel(QLabel):
    def __init__(self, text, parent=None):
        super().__init__(parent)
        self.full_text = text + "   "  # boşluk ekleyerek döngü estetiği sağla
        self.display_text = self.full_text
        self.setText(self.display_text)
        self.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.scrollText)
        self.timer.start(150)  # ms cinsinden (ne kadar düşükse o kadar hızlı kayar)

    def scrollText(self):
        self.display_text = self.display_text[1:] + self.display_text[0]
        self.setText(self.display_text)

class DroneGCS(QWidget):
    def __init__(self):
        # Arayüz ve değişkenleri başlatır
        super().__init__()
        self.initUI()
        self.master = None
        self.drone_thread = None
        self.telemetry_thread = None
        self.waypoints = []
        self.waypoint_markers = []
        self.sonsuz_state = 'idle'
        self.sonsuz_poles = []
        self.sonsuz_pole_markers = []
        self.map_click_handler = MapClickHandler(self)
        self.web_channel = QWebChannel()
        self.web_channel.registerObject('pyqt_webchannel', self.map_click_handler)
        self.web_view.page().setWebChannel(self.web_channel)

    def initUI(self):
        # Arayüz elemanlarını oluşturur ve düzenler
        self.setWindowTitle("🚀 HEZARFEN - Yer Kontrol İstasyonu")
        self.setGeometry(100, 100, 1200, 750)
        self.setStyleSheet("background-color: #E3F2FD;")
        # Uygulama genelinde emoji fontu kullan
        QApplication.setFont(QFont("Noto Color Emoji"))

        # Harita Görüntüleme
        self.web_view = QWebEngineView(self)
        map_path = "/home/aykhan/Desktop/HEZARFEN-main/gcs_project/map.html"
        self.web_view.setUrl(QUrl.fromLocalFile(map_path))
        self.web_view.setStyleSheet("border: 3px solid #B0BEC5; border-radius: 8px;")

        # Durum Bilgisi
        self.status_label = QLabel("🟡 Durum: Bağlı Değil", self)
        self.status_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.status_label.setStyleSheet("font-size: 14px; color: #1E3A5F; background: white; padding: 6px; border-radius: 5px;")

        # Telemetri Verileri
        self.telemetry_group = QGroupBox("📡 Telemetri Verileri")
        self.telemetry_group.setStyleSheet("QGroupBox { font-size: 16px; font-weight: bold; color: #1E3A5F; }")
        telemetry_layout = QVBoxLayout()
        self.lat_label = QLabel("📍 Enlem: -")
        self.lon_label = QLabel("📍 Boylam: -")
        self.alt_label = QLabel("📏 Yükseklik: -")
        self.speed_label = QLabel("💨 Hız: -")
        self.battery_label = QLabel("🔋 Pil: -%")
        self.yaw_label = QLabel("🧭 Yaw Açısı: -")

        for label in [self.lat_label, self.lon_label, self.alt_label, self.speed_label, self.battery_label, self.yaw_label]:
            label.setStyleSheet("font-size: 14px; font-family: 'Noto Color Emoji', 'Segoe UI Emoji', 'Arial'; color: #37474F; padding: 3px;")
            label.setWordWrap(True)

        telemetry_layout.addWidget(self.lat_label)
        telemetry_layout.addWidget(self.lon_label)
        telemetry_layout.addWidget(self.alt_label)
        telemetry_layout.addWidget(self.speed_label)
        telemetry_layout.addWidget(self.battery_label)
        telemetry_layout.addWidget(self.yaw_label)
        self.telemetry_group.setLayout(telemetry_layout)

        # Uçuş Modları & Görev Planlama
        self.flight_modes_group = QGroupBox("✈️ Uçuş Modları & Görev Planlama")
        self.flight_modes_group.setStyleSheet("QGroupBox { font-size: 16px; font-weight: bold; color: #1E3A5F; }")
        flight_layout = QVBoxLayout()
        
        # --- SABİT KANAT MODLARI (SEÇMELİ) ---
        self.flight_mode_combo = QComboBox()
        self.flight_mode_combo.addItems([
            "OTOMATİK UÇUŞ",
            "MANUEL KONTROL",
            "LOİTER MODU",
            "SONSUZ ÇİZDİRME",
            "FBWA",
            "FBWB",
            "CRUISE",
            "STABILIZE",
            "AUTOTUNE",
            "TRAINING",
            "ACRO",
            "QMODES",
            "CIRCLE",
            "GUIDED",
            "RTL",
            "TAKEOFF",
            "THERMAL"
        ])
        self.flight_mode_combo.setStyleSheet("font-size: 14px; padding: 5px; margin-bottom: 5px;")
        self.flight_mode_button = QPushButton("Modu Aktifleştir")
        self.flight_mode_button.setStyleSheet("font-size: 14px; background-color: #1976D2; color: white; padding: 7px; border-radius: 5px; margin-bottom: 8px;")
        def activate_selected_mode():
            idx = self.flight_mode_combo.currentIndex()
            if idx == 0:
                set_auto_mode(self.master, self.waypoints, self.status_label)
            elif idx == 1:
                set_manual_mode()
            elif idx == 2:
                set_loiter_mode()
            elif idx == 3:
                self.handle_sonsuz_button()
            elif idx == 4:
                set_fbwa_mode()
            elif idx == 5:
                set_fbwb_mode()
            elif idx == 6:
                set_cruise_mode()
            elif idx == 7:
                set_stabilize_mode()
            elif idx == 8:
                set_autotune_mode()
            elif idx == 9:
                set_training_mode()
            elif idx == 10:
                set_acro_mode()
            elif idx == 11:
                set_qmodes_mode()
            elif idx == 12:
                set_circle_mode()
            elif idx == 13:
                set_guided_mode()
            elif idx == 14:
                set_rtl_mode()
            elif idx == 15:
                set_takeoff_mode()
            elif idx == 16:
                set_thermal_mode()
        self.flight_mode_button.clicked.connect(activate_selected_mode)
        flight_layout.addWidget(QLabel('<b>Sabit Kanat Modları</b>'))
        flight_layout.addWidget(self.flight_mode_combo)
        flight_layout.addWidget(self.flight_mode_button)

        # --- DÖNER KANAT MODLARI (SEÇMELİ) ---
        self.rotary_mode_combo = QComboBox()
        self.rotary_mode_combo.addItems([
            "ACRO",
            "ALT HOLD",
            "AUTO",
            "AUTOTUNE",
            "BRAKE",
            "CIRCLE",
            "DRIFT",
            "FLIP",
            "FLOWHOLD",
            "FOLLOW",
            "GUIDED",
            "HELI_AUTOROTATE",
            "LAND",
            "LOITER",
            "POSHOLD",
            "RTL",
            "SIMPLE/SUPER SIMPLE",
            "SMARTRTL",
            "SPORT",
            "STABILIZE",
            "SYSID",
            "THROW",
            "TURTLE",
            "ZIGZAG"
        ])
        self.rotary_mode_combo.setStyleSheet("font-size: 14px; padding: 5px; margin-bottom: 5px;")
        self.rotary_mode_button = QPushButton("Modu Aktifleştir")
        self.rotary_mode_button.setStyleSheet("font-size: 14px; background-color: #1976D2; color: white; padding: 7px; border-radius: 5px; margin-bottom: 8px;")
        def activate_selected_rotary_mode():
            idx = self.rotary_mode_combo.currentIndex()
            mode_name = self.rotary_mode_combo.currentText()

            if mode_name == "ACRO":
                set_acro_copter_mode()
            elif mode_name == "ALT HOLD":
                set_alt_hold_mode()
            elif mode_name == "AUTO":
                set_auto_copter_mode()
            elif mode_name == "AUTOTUNE":
                set_autotune_copter_mode()
            elif mode_name == "BRAKE":
                set_brake_mode()
            elif mode_name == "CIRCLE":
                set_circle_copter_mode()
            elif mode_name == "DRIFT":
                set_drift_mode()
            elif mode_name == "FLIP":
                set_flip_mode()
            elif mode_name == "FLOWHOLD":
                set_flowhold_mode()
            elif mode_name == "FOLLOW":
                set_follow_mode()
            elif mode_name == "GUIDED":
                set_guided_copter_mode()
            elif mode_name == "HELI_AUTOROTATE":
                set_heli_autorotate_mode()
            elif mode_name == "LAND":
                set_land_mode()
            elif mode_name == "LOITER":
                set_loiter_copter_mode()
            elif mode_name == "POSHOLD":
                set_poshold_mode()
            elif mode_name == "RTL":
                set_rtl_copter_mode()
            elif mode_name == "SIMPLE/SUPER SIMPLE":
                set_simple_mode()
            elif mode_name == "SMARTRTL":
                set_smartrtl_mode()
            elif mode_name == "SPORT":
                set_sport_mode()
            elif mode_name == "STABILIZE":
                set_stabilize_copter_mode()
            elif mode_name == "SYSID":
                set_sysid_mode()
            elif mode_name == "THROW":
                set_throw_mode()
            elif mode_name == "TURTLE":
                set_turtle_mode()
            elif mode_name == "ZIGZAG":
                set_zigzag_mode()
            else:
                print(f"[Döner Kanat] Seçili mod: {mode_name}")
        self.rotary_mode_button.clicked.connect(activate_selected_rotary_mode)
        flight_layout.addWidget(QLabel('<b>Döner Kanat Modları</b>'))
        flight_layout.addWidget(self.rotary_mode_combo)
        flight_layout.addWidget(self.rotary_mode_button)
        self.start_infinity_button = QPushButton("Sonsuzu Başlat", self)
        self.start_infinity_button.setEnabled(True)
        self.start_infinity_button.setVisible(True)
        self.start_infinity_button.clicked.connect(self.handle_start_infinity_button)
        self.add_waypoint_button = QPushButton("📍 Waypoint Ekle", self)
        self.clear_mission_button = QPushButton("🗑️ Görevi Temizle", self)
        self.add_waypoint_button.setStyleSheet("font-size: 14px; background-color: #388E3C; color: white; padding: 7px; border-radius: 5px;")
        self.clear_mission_button.setStyleSheet("font-size: 14px; background-color: #388E3C; color: white; padding: 7px; border-radius: 5px;")
        self.add_waypoint_button.clicked.connect(self.add_waypoint)
        self.clear_mission_button.clicked.connect(self.clear_mission)
        flight_layout.addWidget(self.start_infinity_button)
        flight_layout.addWidget(self.add_waypoint_button)
        flight_layout.addWidget(self.clear_mission_button)
        
        # Waypoint listesi
        self.waypoint_list = QListWidget()
        self.waypoint_list.setFixedHeight(120)
        self.waypoint_list.setStyleSheet("font-size: 12px; background-color: #FAFAFA; border: 1px solid #BDBDBD;")
        self.waypoint_list.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        flight_layout.addWidget(self.waypoint_list)
        
        # Waypoint yükseklik düzenleme (çift tık ile)
        self.waypoint_list.itemDoubleClicked.connect(self.edit_waypoint_altitude)
        
        self.flight_modes_group.setLayout(flight_layout)

        # Bağlantı & Acil Durum
        self.connect_button = MarqueeLabel("🔗 Bağlan")
        self.connect_button.setStyleSheet("font-size: 16px; background-color: #4CAF50; color: white; padding: 10px; border-radius: 8px; font-weight: bold; border: 2px solid #388E3C;")
        self.connect_button.mousePressEvent = lambda event: self.connect_to_drone()
        self.start_telemetry_button = MarqueeLabel("📡 Telemetriyi Başlat")
        self.start_telemetry_button.setStyleSheet("font-size: 14px; background-color: #2196F3; color: white; padding: 8px; border-radius: 6px; border: 2px solid #1976D2;")
        self.start_telemetry_button.setEnabled(False)
        self.start_telemetry_button.mousePressEvent = lambda event: self.start_telemetry() if self.start_telemetry_button.isEnabled() else None
        
        self.emergency_button = MarqueeLabel("🆘 Acil Durdurma")
        self.emergency_button.setStyleSheet("font-size: 14px; background-color: #D32F2F; color: white; font-weight: bold; padding: 8px; border-radius: 5px; border: 2px solid #B71C1C;")
        self.emergency_button.mousePressEvent = lambda event: self.emergency_stop()

        # Bağlantı Ayarları Grubu
        self.connection_group = QGroupBox("🔧 Bağlantı Ayarları")
        self.connection_group.setStyleSheet("QGroupBox { font-size: 16px; font-weight: bold; color: #1E3A5F; }")

        # Port ve Baudrate combo box
        self.port_combo = QComboBox()
        self.port_combo.addItems([
            "udp:127.0.0.1:14550",  # ArduPilot SITL varsayılan port
            "udp:14550",
            "/dev/ttyUSB0",
            "/dev/ttyACM0",
            "COM3",
            "tcp:192.168.2.1:5760"
        ])
        self.port_combo.setCurrentText("udp:127.0.0.1:14550")
        self.port_combo.setEditable(True)
        
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "921600"])
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setEditable(True)

        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("🔌 Port:"))
        conn_layout.addWidget(self.port_combo)
        conn_layout.addWidget(QLabel("⚙️ Baud:"))
        conn_layout.addWidget(self.baud_combo)

        self.connection_group.setLayout(conn_layout)

        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.web_view, stretch=2)
        
        right_layout = QVBoxLayout()
        
        right_layout.addWidget(self.status_label)
        
        right_layout.addWidget(self.connection_group)
        
        connection_layout = QHBoxLayout()
        connection_layout.addWidget(self.connect_button)
        connection_layout.addWidget(self.start_telemetry_button)
        connection_layout.addWidget(self.emergency_button)
        right_layout.addLayout(connection_layout)
        
        right_layout.addWidget(self.telemetry_group)
        
        right_layout.addWidget(self.flight_modes_group)

        # Sağ paneli sabit genişlikte bir QWidget'e koy
        right_panel = QWidget()
        right_panel.setLayout(right_layout)
        right_panel.setFixedWidth(400)  # Sağ panel genişliği (isteğe göre değiştirilebilir)

        main_layout.addLayout(left_layout)
        main_layout.addWidget(right_panel)
        self.setLayout(main_layout)

        self.web_view.page().loadFinished.connect(self.setup_map_click_handler)

    def connect_to_drone(self):
        # Drone bağlantısını başlatır
        self.connect_button.setEnabled(False)
        self.status_label.setText("🔄 Bağlanıyor...")

        raw_input = self.port_combo.currentText().strip()
        baudrate = int(self.baud_combo.currentText())

        if raw_input.startswith("udp:") and raw_input.count(":") == 1:
            port = raw_input.split(":")[1]
            connection_string = f"udp:127.0.0.1:{port}"
        elif raw_input.startswith("udp:") and raw_input.count(":") == 2:
            connection_string = raw_input
        else:
            connection_string = raw_input

        print(f"🔌 Bağlanıyor: {connection_string} @ {baudrate}")

        self.drone_thread = DroneConnectionThread(connection_string, baudrate)
        self.drone_thread.connection_status.connect(self.update_status)
        self.drone_thread.start()

    def update_status(self, success, master):
        # Bağlantı durumunu günceller
        if success:
            self.status_label.setText("✅ Durum: Bağlandı")
            self.master = master  
            self.start_telemetry_button.setEnabled(True)
            # MarqueeLabel için metin güncelleme
            self.connect_button.full_text = "🔌 Bağlantıyı Kes   "
            self.connect_button.display_text = self.connect_button.full_text
            self.connect_button.setText(self.connect_button.display_text)
            self.connect_button.mousePressEvent = lambda event: self.disconnect_from_drone()
        else:
            self.status_label.setText("❌ Bağlantı Hatası")
            self.connect_button.setEnabled(True)
            # MarqueeLabel için metin güncelleme
            self.connect_button.full_text = "🔗 Bağlan   "
            self.connect_button.display_text = self.connect_button.full_text
            self.connect_button.setText(self.connect_button.display_text)
            self.connect_button.mousePressEvent = lambda event: self.connect_to_drone()

    def disconnect_from_drone(self):
        # Drone bağlantısını keser
        if self.master:
            try:
                self.master.close()
                self.master = None
                self.status_label.setText("🟡 Durum: Bağlı Değil")
                self.start_telemetry_button.setEnabled(False)
                # MarqueeLabel için metin güncelleme
                self.connect_button.full_text = "🔗 Bağlan   "
                self.connect_button.display_text = self.connect_button.full_text
                self.connect_button.setText(self.connect_button.display_text)
                self.connect_button.mousePressEvent = lambda event: self.connect_to_drone()
                print("🔌 Bağlantı kesildi")
            except Exception as e:
                print(f"❌ Bağlantı kesme hatası: {e}")

    def start_telemetry(self):
        # Telemetri verilerini başlatır
        if self.master:
            self.status_label.setText("📡 Telemetri Başladı...")
            
            self.web_view.page().runJavaScript("""
                if (typeof trailPoints !== 'undefined') {
                    trailPoints = [];
                    droneTrail.setLatLngs([]);
                }
                mapInitialized = false;
                map.setView([39.9334, 32.8597], 6);
                console.log('Harita Türkiye\'ye döndü, drone konumu bekleniyor...');
            """)
            
            self.telemetry_thread = TelemetryThread(self.master)
            self.telemetry_thread.telemetry_signal.connect(self.update_telemetry)
            self.telemetry_thread.start()
        else:
            self.status_label.setText("❌ Telemetri Başlatılamadı!")

    def update_telemetry(self, data):
        # Telemetri verilerini arayüzde günceller
        if 'lat' in data and 'lon' in data:
            lat, lon = data['lat'], data['lon']
            alt = data.get('alt', 0)
            speed = data.get('speed', 0)
            battery = data.get('battery', 0)
            yaw = data.get('yaw', 0)

            self.lat_label.setText(f"📍 Enlem: {lat:.6f}°")
            self.lon_label.setText(f"📍 Boylam: {lon:.6f}°")
            self.alt_label.setText(f"📏 Yükseklik: {alt:.1f} m")
            self.speed_label.setText(f"💨 Hız: {speed:.1f} m/s")
            self.battery_label.setText(f"🔋 Pil: {battery:.1f}%")
            self.yaw_label.setText(f"🧭 Yaw Açısı: {yaw:.1f}°")

            print(f"📍 Harita Güncelleniyor: {lat:.6f}, {lon:.6f}")
            
            js_code = f"updateDronePosition({lat}, {lon});"
            self.web_view.page().runJavaScript(js_code)

    def add_waypoint(self):
        # Manuel waypoint ekleme işlemini başlatır
        """Manuel waypoint ekleme (şimdilik harita tıklama ile çalışıyor)"""
        print("📍 Harita üzerinde bir yere tıklayarak waypoint ekleyebilirsiniz!")
        self.status_label.setText("📍 Harita üzerinde bir yere tıklayın!")

    def clear_mission(self):
        # Tüm waypoint'leri ve markerları temizler
        """Tüm waypoint'leri temizle"""
        self.waypoints = []
        self.waypoint_list.clear()
        for marker_id in self.waypoint_markers:
            self.web_view.page().runJavaScript(f"if (window.{marker_id}) {{ map.removeLayer(window.{marker_id}); window.{marker_id} = null; }}")
        self.waypoint_markers = []
        print("🗑️ Tüm waypoint'ler temizlendi")
        self.status_label.setText("🗑️ Görev temizlendi")

    def emergency_stop(self):
        # Acil durdurma işlemini gerçekleştirir
        self.status_label.setText("🛑 Acil Durum Aktif! IHA Durduruldu!")
        print("🚨 Acil durdurma işlemi başlatıldı!")
        if self.telemetry_thread:
            try:
                self.telemetry_thread.stop()
            except Exception as e:
                print(f"Telemetri thread durdurulurken hata: {e}")
            self.telemetry_thread = None
        if self.master:
            try:
                self.master.close()
            except Exception as e:
                print(f"Drone bağlantısı kapatılırken hata: {e}")
            self.master = None
        self.start_telemetry_button.setEnabled(False)
        # MarqueeLabel için metin güncelleme
        self.connect_button.full_text = "🔗 Bağlan   "
        self.connect_button.display_text = self.connect_button.full_text
        self.connect_button.setText(self.connect_button.display_text)
        self.connect_button.mousePressEvent = lambda event: self.connect_to_drone()
        self.lat_label.setText("📍 Enlem: -")
        self.lon_label.setText("📍 Boylam: -")
        self.alt_label.setText("📏 Yükseklik: -")
        self.speed_label.setText("💨 Hız: -")
        self.battery_label.setText("🔋 Pil: -%")
        self.yaw_label.setText("🧭 Yaw Açısı: -")
        self.web_view.page().runJavaScript("""
            if (typeof trailPoints !== 'undefined') {
                trailPoints = [];
                droneTrail.setLatLngs([]);
            }
            mapInitialized = false;
            map.setView([39.9334, 32.8597], 6);
        """)
        self.waypoints = []
        self.waypoint_list.clear()
        for marker_id in self.waypoint_markers:
            self.web_view.page().runJavaScript(f"if (window.{marker_id}) {{ map.removeLayer(window.{marker_id}); window.{marker_id} = null; }}")
        self.waypoint_markers = []
        print("🔌 Acil durdurma: Tüm bağlantılar ve threadler kapatıldı.")

    def handle_sonsuz_button(self):
        # Sonsuz modunu başlatır veya bekler
        if self.sonsuz_state == 'idle':
            print('[SONSUZ] Mod başlatıldı, direk seçimi bekleniyor.')
            self.status_label.setText("♾️ Direklerin konumlarını belirleyin (haritada iki nokta seçin)")
            self.sonsuz_poles = []
            self.sonsuz_pole_markers = []
            self.sonsuz_state = 'selecting'

    def handle_start_infinity_button(self):
        # Sonsuz görevini başlatır
        print(f'[SONSUZ] Sonsuzu Başlat butonuna tıklandı, state: {self.sonsuz_state}, sonsuz_poles: {getattr(self, "sonsuz_poles", None)} (len={len(getattr(self, "sonsuz_poles", []))})')
        if hasattr(self, 'sonsuz_poles') and len(self.sonsuz_poles) == 2:
            self.waypoints = generate_infinity_waypoints(self.sonsuz_poles)
            start_infinity_mission(self.master, self.waypoints, self.status_label)
            self.sonsuz_state = 'idle'
            self.status_label.setText("♾️ Sonsuz görev başlatıldı!")
        else:
            self.status_label.setText("❗ Lütfen önce iki direk seçin!")

    def setup_map_click_handler(self):
        # Harita tıklama olayını ayarlar
        """Harita yüklendikten sonra tıklama olayını ayarla"""
        self.web_view.page().runJavaScript("""
            // WebChannel'ı yükle
            new QWebChannel(qt.webChannelTransport, function(channel) {
                window.pyqt_webchannel = channel.objects.pyqt_webchannel;
            });
            
            map.on('click', function(e) {
                var lat = e.latlng.lat;
                var lon = e.latlng.lng;
                // Python'a veri gönder
                if (window.pyqt_webchannel) {
                    window.pyqt_webchannel.handle_message(JSON.stringify({
                        type: 'map_click',
                        lat: lat,
                        lon: lon
                    }));
                }
            });
        """)
        
        self.web_view.page().javaScriptConsoleMessage.connect(self.handle_map_click)
    
    def handle_map_click(self, level, message, line, source):
        # Harita tıklama mesajlarını işler
        print(f'[SONSUZ] handle_map_click çağrıldı, sonsuz_state: {getattr(self, "sonsuz_state", None)}')
        if "MAP_CLICK" in message:
            try:
                coords = message.split("MAP_CLICK:")[1].split(",")
                lat = float(coords[0])
                lon = float(coords[1])
                
                alt, ok = QInputDialog.getDouble(self, "Yükseklik Belirle", "Waypoint yüksekliği (m):", 50.0, 0, 10000, 1)
                if not ok:
                    return
                
                self.add_waypoint_from_map(lat, lon, alt)
                
            except Exception as e:
                print(f"Harita tıklama işlenirken hata: {e}")
    
    def add_waypoint_from_map(self, lat, lon, alt):
        # Haritadan waypoint ekler
        """Haritadan waypoint ekle"""
        self.waypoints.append((lat, lon, alt))
        item_text = f"{len(self.waypoints)}. WP: {lat:.6f}, {lon:.6f}, {alt:.1f}m"
        item = QListWidgetItem(item_text)
        self.waypoint_list.addItem(item)
        marker_id = f"wp_marker_{len(self.waypoints)}"
        self.waypoint_markers.append(marker_id)
        self.web_view.page().runJavaScript(f"""
            window.{marker_id} = L.marker([{lat}, {lon}], {{
                icon: L.divIcon({{
                    className: 'waypoint-marker',
                    html: '<div style=\"background-color: #FF5722; color: white; border-radius: 50%; width: 20px; height: 20px; text-align: center; line-height: 20px; font-size: 12px; font-weight: bold;\">{len(self.waypoints)}</div>',
                    iconSize: [20, 20],
                    iconAnchor: [10, 10]
                }})
            }}).addTo(map);
        """)

    def edit_waypoint_altitude(self, item):
        # Waypoint yüksekliğini düzenler
        """Waypoint yüksekliğini düzenle"""
        index = self.waypoint_list.row(item)
        if 0 <= index < len(self.waypoints):
            lat, lon, alt = self.waypoints[index]
            new_alt, ok = QInputDialog.getDouble(self, "Yüksekliği Düzenle", f"{index+1}. Waypoint için yeni yükseklik (m):", alt, 0, 10000, 1)
            if ok:
                self.waypoints[index] = (lat, lon, new_alt)
                item.setText(f"{index+1}. WP: {lat:.6f}, {lon:.6f}, {new_alt:.1f}m")

    def closeEvent(self, event):
        # Pencere kapatılırken threadleri durdurur
        if self.telemetry_thread:
            self.telemetry_thread.stop()
        if self.drone_thread:
            self.drone_thread.quit()
            self.drone_thread.wait()
        event.accept()
        if self.drone_thread:
            self.drone_thread.quit()
            self.drone_thread.wait()
