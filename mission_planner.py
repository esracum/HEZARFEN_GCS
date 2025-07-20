def send_waypoints_to_drone(master, waypoints):
    """Verilen master ve waypoints ile drone'a görev yükler."""
    try:
        # Mevcut mission'ı temizle
        master.clear_mission()
        # Waypoint'leri ekle
        for i, (lat, lon, alt) in enumerate(waypoints):
            wp = master.commands.add()
            wp.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            wp.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp.param1 = 0  # Hold time
            wp.param2 = 10  # Acceptance radius
            wp.param3 = 0  # Pass radius
            wp.param4 = 0  # Yaw
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = alt
            wp.autocontinue = 1
            wp.current = 0
        # Mission'ı drone'a gönder
        master.commands.upload()
        print(f"📡 {len(waypoints)} waypoint drone'a gönderildi")
    except Exception as e:
        print(f"❌ Waypoint gönderilirken hata: {e}")
        raise e
