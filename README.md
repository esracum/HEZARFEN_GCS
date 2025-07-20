# Hezarfen Takımı Ground Control System (Yer Kontrol İstasyonu)

📡 **Hezarfen Takımı GCS (Ground Control Station)**, İHA'ların yerden kontrolü, canlı telemetri takibi, kamera görüntüsü aktarımı ve görev planlama işlevleri için geliştirilmiş bir **Python / PyQt5 tabanlı** masaüstü uygulamasıdır. 
- 👨‍💻 **Geliştiriciler:** Aykhan Shirinzade, Esra Cüm

## 🚀 Özellikler
- 📡 **Telemetri İzleme:** Anlık hız, irtifa, yön, konum gibi uçuş verilerinin takibi.
- 🎥 **Canlı Kamera Görüntüsü:** Araç üzerindeki kamera akışınn gerçek zamanlı görüntülenmesi.
- 🗺️ **Harita Üzerinde Gösterim:** GPS verilerini harita üzerinden takip etme.
- 🛫 **Görev Planlama:** Önceden belirlenen rotaların yüklenmesi ve görevin başlatılması.
- 📊 **Veri Kayıt:** Telemetri ve diğer verileri anlık olarak kaydetme.
- 🖥️ **Arayüz:** PyQt5 ile geliştirilmiş arayüz.
- 📡 **Seri veya UDP İletişim Desteği:** Araç ile farklı bağlantı yöntemleri üzerinden veri alışverişi.

## ⚙️ Kurulum
```bash
git clone https://github.com/hezarfen-takimi/gcs.git
cd gcs
pip install -r requirements.txt
