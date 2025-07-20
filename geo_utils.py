import math

def haversine(lon1, lat1, lon2, lat2):
    """İki koordinat arasındaki mesafeyi (metre cinsinden) hesaplar."""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def generate_infinity_waypoints(poles, alt=50.0):
    """
    Sonsuz (∞) şekli için waypoint listesi üretir.
    poles: [(lat1, lon1), (lat2, lon2)]
    alt: Yükseklik (metre)
    """
    (lat1, lon1), (lat2, lon2) = poles
    print(f"1 -> {lat1}, {lon1}")
    print(f"2 -> {lat2}, {lon2}")
    mesafe = haversine(lon1, lat1, lon2, lat2)
    aralik_mesafesi = mesafe / 2
    dx = lon2 - lon1
    dy = lat2 - lat1
    norm = (dx**2 + dy**2) ** 0.5
    ux = dx / norm
    uy = dy / norm
    vx = -uy
    vy = ux
    wp1_lat = lat1 + vy * aralik_mesafesi / 111320
    wp1_lon = lon1 + vx * aralik_mesafesi / (111320 * math.cos(math.radians(lat1)))
    ortalat = (lat1 + lat2) / 2
    ortalon = (lon1 + lon2) / 2
    wp3_lat = lat2 - vy * aralik_mesafesi / 111320
    wp3_lon = lon2 - vx * aralik_mesafesi / (111320 * math.cos(math.radians(lat2)))
    wp4_lat = lat2 + ux * aralik_mesafesi / 111320
    wp4_lon = lon2 + uy * aralik_mesafesi / (111320 * math.cos(math.radians(lat2)))
    wp5_lat = lat2 + vy * aralik_mesafesi / 111320
    wp5_lon = lon2 + vx * aralik_mesafesi / (111320 * math.cos(math.radians(lat2)))
    wp7_lat = lat1 - vy * aralik_mesafesi / 111320
    wp7_lon = lon1 - vx * aralik_mesafesi / (111320 * math.cos(math.radians(lat1)))
    wp8_lat = lat1 - ux * aralik_mesafesi / 111320
    wp8_lon = lon1 - uy * aralik_mesafesi / (111320 * math.cos(math.radians(lat1)))
    waypoints = [
        (wp8_lat, wp8_lon, alt),
        (wp1_lat, wp1_lon, alt),
        (ortalat, ortalon, alt),
        (wp3_lat, wp3_lon, alt),
        (wp4_lat, wp4_lon, alt),
        (wp5_lat, wp5_lon, alt),
        (ortalat, ortalon, alt),
        (wp7_lat, wp7_lon, alt),
        (wp8_lat, wp8_lon, alt),
        (wp1_lat, wp1_lon, alt),
        (ortalat, ortalon, alt),
        (wp3_lat, wp3_lon, alt),
        (wp4_lat, wp4_lon, alt),
        (wp5_lat, wp5_lon, alt),
        (ortalat, ortalon, alt),
        (wp7_lat, wp7_lon, alt),
        (lat1, lon1, alt)
    ]
    return waypoints 