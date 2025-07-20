import time
from dronekit import VehicleMode, Command
from pymavlink import mavutil
import math
from geo_utils import haversine

class FlightModes:
    def __init__(self, mav_connection):
        self.master = mav_connection

    def set_mode(self, mode):
        mode_mapping = {
            "STABILIZE": 0,
            "AUTO": 3,
            "RTL": 6,
            "LAND": 9
        }
        mode_id = mode_mapping.get(mode)
        if mode_id is not None:
            self.master.set_mode(mode_id)
            print(f"Uçuş modu {mode} olarak değiştirildi.")

def start_infinity_mission(master, waypoints, status_label):
    """FBWA ile kalkış ve ardından AUTO ile sonsuz görev başlatır."""
    try:
        if not master:
            status_label.setText("❌ Drone bağlı değil!")
            return
        if not waypoints:
            status_label.setText("❌ Waypoint bulunamadı!")
            return
        master.commands.clear()
        master.flush()
        cmds = master.commands
        cmds.clear()
        for lat, lon, alt in waypoints:
            cmds.add(Command(
                0, 0, 0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0, 0, 0, 0, 0,
                lat, lon, alt
            ))
        # RTL komutu ekle
        cmds.add(Command(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0,
            0, 0, 0
        ))
        cmds.upload()
        master.mode = VehicleMode('FBWA')
        status_label.setText("FBWA moduna geçiliyor...")
        time.sleep(2)
        if not master.armed:
            master.armed = True
            status_label.setText("🔄 ARM ediliyor...")
            while not master.armed:
                time.sleep(1)
        master.channels.overrides['3'] = 1800
        status_label.setText("Throttle veriliyor (RC3=1800)...")
        time.sleep(4)
        master.mode = VehicleMode('AUTO')
        status_label.setText("🚀 Sonsuz görev başlatıldı!")
        master.channels.overrides['3'] = None
    except Exception as e:
        status_label.setText(f"❌ Hata: {e}")

def clear_sonsuz_pole_markers(web_view, sonsuz_pole_markers):
    if sonsuz_pole_markers:
        for marker_id in sonsuz_pole_markers:
            web_view.page().runJavaScript(f"if (window.{marker_id}) {{ map.removeLayer(window.{marker_id}); window.{marker_id} = null; }}")
        sonsuz_pole_markers.clear()

# AUTO modunu eski haliyle bırakıyorum, diğer modlar sadece print yapacak

def set_manual_mode(*args, **kwargs):
    print("MANUAL mode selected")
def set_fbwa_mode(*args, **kwargs):
    print("FBWA mode selected")
def set_fbwb_mode(*args, **kwargs):
    print("FBWB mode selected")
def set_cruise_mode(*args, **kwargs):
    print("CRUISE mode selected")
def set_stabilize_mode(*args, **kwargs):
    print("STABILIZE mode selected")
def set_autotune_mode(*args, **kwargs):
    print("AUTOTUNE mode selected")
def set_training_mode(*args, **kwargs):
    print("TRAINING mode selected")
def set_acro_mode(*args, **kwargs):
    print("ACRO mode selected")
def set_qmodes_mode(*args, **kwargs):
    print("QMODES (QCopter Modes) selected")
def set_loiter_mode(*args, **kwargs):
    print("LOITER mode selected")
def set_circle_mode(*args, **kwargs):
    print("CIRCLE mode selected")
def set_guided_mode(*args, **kwargs):
    print("GUIDED mode selected")
def set_rtl_mode(*args, **kwargs):
    print("RTL (Return To Launch) mode selected")
def set_takeoff_mode(*args, **kwargs):
    print("TAKEOFF mode selected")
def set_thermal_mode(*args, **kwargs):
    print("THERMAL mode selected")

def set_auto_mode(master, waypoints, status_label):
    if not master:
        status_label.setText("❌ Drone bağlı değil!")
        return
    if not waypoints:
        status_label.setText("❌ Waypoint bulunamadı!")
        return
    try:
        master.commands.clear()
        master.flush()
        cmds = master.commands
        cmds.clear()
        for lat, lon, alt in waypoints:
            cmds.add(Command(0, 0, 0, 3, 16, 0, 0, 0, 0, 0, 0, lat, lon, alt))
        cmds.upload()
        print(f"📡 {len(waypoints)} waypoint drone'a yüklendi")

        master.mode = VehicleMode('FBWA')
        status_label.setText("FBWA moduna geçiliyor...")
        print("FBWA moduna geçiliyor...")
        time.sleep(2)

        if not master.armed:
            master.armed = True
            status_label.setText("🔄 ARM ediliyor...")
            print("🔄 ARM ediliyor...")
            while not master.armed:
                time.sleep(1)

        master.channels.overrides['3'] = 1800
        status_label.setText("Throttle veriliyor (RC3=1800)...")
        print("Throttle veriliyor (RC3=1800)...")
        time.sleep(4)

        master.mode = VehicleMode('AUTO')
        status_label.setText("🚀 Otomatik uçuş başlatıldı!")
        print("🚀 AUTO moduna geçildi ve görev başlatıldı")

        master.channels.overrides['3'] = None

    except Exception as e:
        print(f"❌ Otomatik uçuş başlatılırken hata: {e}")
        status_label.setText(f"❌ Hata: {e}")

def set_auto_mode_print(*args, **kwargs):
    print("AUTO mode selected (print only)")

def set_acro_copter_mode(*args, **kwargs):
    print("ACRO (Copter) mode selected")
def set_alt_hold_mode(*args, **kwargs):
    print("ALT HOLD mode selected")
def set_auto_copter_mode(*args, **kwargs):
    print("AUTO (Copter) mode selected")
def set_autotune_copter_mode(*args, **kwargs):
    print("AUTOTUNE (Copter) mode selected")
def set_brake_mode(*args, **kwargs):
    print("BRAKE mode selected")
def set_circle_copter_mode(*args, **kwargs):
    print("CIRCLE (Copter) mode selected")
def set_drift_mode(*args, **kwargs):
    print("DRIFT mode selected")
def set_flip_mode(*args, **kwargs):
    print("FLIP mode selected")
def set_flowhold_mode(*args, **kwargs):
    print("FLOWHOLD mode selected")
def set_follow_mode(*args, **kwargs):
    print("FOLLOW mode selected")
def set_guided_copter_mode(*args, **kwargs):
    print("GUIDED (Copter) mode selected")
def set_heli_autorotate_mode(*args, **kwargs):
    print("HELI AUTOROTATE mode selected")
def set_land_mode(*args, **kwargs):
    print("LAND mode selected")
def set_loiter_copter_mode(*args, **kwargs):
    print("LOITER (Copter) mode selected")
def set_poshold_mode(*args, **kwargs):
    print("POSHOLD mode selected")
def set_rtl_copter_mode(*args, **kwargs):
    print("RTL (Copter) mode selected")
def set_simple_mode(*args, **kwargs):
    print("SIMPLE/SUPER SIMPLE mode selected")
def set_smartrtl_mode(*args, **kwargs):
    print("SMARTRTL mode selected")
def set_sport_mode(*args, **kwargs):
    print("SPORT mode selected")
def set_stabilize_copter_mode(*args, **kwargs):
    print("STABILIZE (Copter) mode selected")
def set_sysid_mode(*args, **kwargs):
    print("SYSID mode selected")
def set_throw_mode(*args, **kwargs):
    print("THROW mode selected")
def set_turtle_mode(*args, **kwargs):
    print("TURTLE mode selected")
def set_zigzag_mode(*args, **kwargs):
    print("ZIGZAG mode selected")