import cv2
import rasterio
import numpy as np
import serial
import serial.tools.list_ports
import time
import datetime
from matplotlib import pyplot as plt


FULL_IMG   = 'america.tif'
CROP_IMG   = 'test.png'

UART_BAUD_RATE  = 9600 
ALTITUDE   = 100.0 # In meters it is stable for this code (needs to be same as original GPS on drone)
INTERVAL   = 1.0 # 1Hz is acceptable for Ardupilot and lowers proccessing power

# Match crop image against full GeoTIFF and return lat/lon of the found location
# Needs to be chnaged to camera feed
# Detection algorithm will be affected based on rotaion if no gimbal is used
# These parameters worked well but more comparing is needed from my side
def find_location(full_path, crop_path, detector_type="ORB", matcher_type="BF", ratio_thresh=0.75, min_match_count=10):
    with rasterio.open(full_path) as src:
        rgb = np.transpose(src.read([1, 2, 3]), (1, 2, 0)).astype(np.uint8)
        tf  = src.transform
    gray_full = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

    crop_bgr  = cv2.imread(crop_path)
    if crop_bgr is None:
        raise ValueError(f"Could not load crop image: {crop_path}")
    crop_rgb  = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
    gray_crop = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)

    if detector_type == "SIFT":
        det = cv2.SIFT_create()
    elif detector_type == "ORB":
        det = cv2.ORB_create(nfeatures=2000)
    elif detector_type == "AKAZE":
        det = cv2.AKAZE_create()
    else:
        raise ValueError(f"Unsupported detector_type: {detector_type}")

    kp1, des1 = det.detectAndCompute(gray_crop, None)
    kp2, des2 = det.detectAndCompute(gray_full, None)

    if des1 is None or des2 is None:
        return None

    if matcher_type == "BF":
        norm = cv2.NORM_L2 if detector_type == "SIFT" else cv2.NORM_HAMMING
        matcher = cv2.BFMatcher(norm, crossCheck=False)
    elif matcher_type == "FLANN":
        if detector_type == "SIFT":
            index_params = dict(algorithm=1, trees=5)
        else:
            index_params = dict(algorithm=6, table_number=6, key_size=12, multi_probe_level=1)
        matcher = cv2.FlannBasedMatcher(index_params, dict(checks=50))
    else:
        raise ValueError(f"Unsupported matcher_type: {matcher_type}")

    matches = matcher.knnMatch(des1, des2, k=2)
    good = [m for m, n in matches if m.distance < ratio_thresh * n.distance]

    if len(good) < min_match_count:
        print(f"not enough matches ({len(good)}/{min_match_count})")
        return None

    src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    if M is None:
        return None

    h, w = gray_crop.shape
    corners = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
    box = cv2.perspectiveTransform(corners, M)

    px = int(np.min(box[:,0,0]))
    py = int(np.min(box[:,0,1]))
    lon, lat = tf * (px, py)

    img_box = rgb.copy()
    cv2.polylines(img_box, [np.int32(box)], True, (0,255,255), 3, cv2.LINE_AA)

    img_matches = cv2.drawMatches(
        crop_rgb, kp1, rgb, kp2, good, None,
        matchColor=(0,255,0), singlePointColor=(0,0,255),
        matchesMask=mask.ravel().tolist(),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )

    return lat, lon, np.sum(mask)/len(good), img_matches, img_box

# XOR checksum is used for NMEA, if UBX change to Fletcher-16 
def checksum(s):
    v = 0
    for c in s:
        v ^= ord(c)
    return f"{v:02X}"

def fmt_lat(dd):
    d = int(abs(dd))
    m = (abs(dd) - d) * 60
    return f"{d:02d}{m:07.4f}", 'N' if dd >= 0 else 'S'

def fmt_lon(dd):
    d = int(abs(dd))
    m = (abs(dd) - d) * 60
    return f"{d:03d}{m:07.4f}", 'E' if dd >= 0 else 'W'

def gpgga(lat, lon, alt):
    t = datetime.datetime.now(datetime.timezone.utc).strftime("%H%M%S.%f")[:-3]
    la, ld = fmt_lat(lat)
    lo, od = fmt_lon(lon)
    b = f"GPGGA,{t},{la},{ld},{lo},{od},1,08,1.0,{alt:.1f},M,,M,,"
    return f"${b}*{checksum(b)}\r\n"

def gprmc(lat, lon):
    now = datetime.datetime.now(datetime.timezone.utc)
    t = now.strftime("%H%M%S.%f")[:-3]
    d = now.strftime("%d%m%y")
    la, ld = fmt_lat(lat)
    lo, od = fmt_lon(lon)
    b = f"GPRMC,{t},A,{la},{ld},{lo},{od},0.0,0.0,{d},,"
    return f"${b}*{checksum(b)}\r\n"

# This will depend on your TTL/USB Converter
def pick_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("no ports found")
        return None
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
    cp210x = [p for p in ports if "CP210" in (p.description or "")]
    if cp210x:
        return cp210x[0].device
    if len(ports) == 1:
        return ports[0].device
    while True:
        try:
            c = int(input(f"select [0-{len(ports)-1}]: "))
            if 0 <= c < len(ports):
                return ports[c].device
        except ValueError:
            pass


def main():
    print(f"matching {CROP_IMG} in {FULL_IMG}...")
    res = find_location(FULL_IMG, CROP_IMG, detector_type="ORB", matcher_type="BF", ratio_thresh=0.75, min_match_count=10)
    if res is None:
        print("no match found")
        return

    lat, lon, conf, img_matches, img_box = res
    print(f"lat={lat:.6f} lon={lon:.6f} confidence={conf:.2f}")

    plt.figure(figsize=(15, 7))
    plt.subplot(121), plt.imshow(img_matches), plt.title("matches")
    plt.subplot(122), plt.imshow(img_box), plt.title("location")
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(2)

    port = pick_port()
    if not port:
        return

    ser = serial.Serial(port, UART_BAUD_RATE, timeout=1)
    print(f"sending to {port} at 1Hz (Ctrl+C to stop)")
    try:
        while True:
            a = gpgga(lat, lon, ALTITUDE)
            b = gprmc(lat, lon)
            ser.write(a.encode('ascii'))
            ser.write(b.encode('ascii'))
            print(a.strip())
            print(b.strip())
            time.sleep(INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
