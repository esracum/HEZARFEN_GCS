from PyQt5.QtCore import QObject, pyqtSlot

class MapClickHandler(QObject):
    """Harita tıklama olaylarını işle"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
    
    @pyqtSlot(str)
    def handle_message(self, message):
        """JavaScript'ten gelen mesajı işle"""
        try:
            import json
            data = json.loads(message)
            if data.get('type') == 'map_click':
                lat = data['lat']
                lon = data['lon']
                if self.parent:
                    # Sonsuz mod aktifse burada direk ekle
                    if hasattr(self.parent, 'sonsuz_state') and self.parent.sonsuz_state in ['selecting', 'ready']:
                        self.parent.sonsuz_poles.append((lat, lon))
                        pole_idx = len(self.parent.sonsuz_poles)
                        marker_id = f"sonsuz_pole_marker_{pole_idx}"
                        self.parent.sonsuz_pole_markers.append(marker_id)
                        color = '#009688' if pole_idx == 1 else '#FFC107'
                        label = f"D{pole_idx}"
                        self.parent.web_view.page().runJavaScript(f"""
                            window.{marker_id} = L.marker([{lat}, {lon}], {{
                                icon: L.divIcon({{
                                    className: 'sonsuz-pole-marker',
                                    html: '<div style=\"background-color: {color}; color: white; border-radius: 50%; width: 28px; height: 28px; text-align: center; line-height: 28px; font-size: 15px; font-weight: bold; border: 2px solid #333;\">{label}</div>',
                                    iconSize: [28, 28],
                                    iconAnchor: [14, 14]
                                }})
                            }}).addTo(map);
                        """)
                        print(f'[SONSUZ][QWebChannel] Direk seçildi: {lat}, {lon} (toplam: {len(self.parent.sonsuz_poles)}) -> {self.parent.sonsuz_poles}')
                        if len(self.parent.sonsuz_poles) == 2:
                            self.parent.sonsuz_state = 'ready'
                            self.parent.status_label.setText("♾️ Direkler seçildi. Sonsuzu Başlat'a tıklayın!")
                        else:
                            self.parent.status_label.setText(f"♾️ {len(self.parent.sonsuz_poles)}. direk seçildi, bir tane daha seçin.")
                        return
                    # Sonsuz mod değilse normal waypoint ekle
                    self.parent.add_waypoint_from_map(lat, lon, 50.0)
        except Exception as e:
            print(f"Harita tıklama mesajı işlenirken hata: {e}") 