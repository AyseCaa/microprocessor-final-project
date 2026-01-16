"""
YOLO tabanlı araç sayımı + PIC16F1719'e UART ile yoğunluk gönderimi.

UART protokolü: C:<nn>\n  (nn = 00..99)
PIC tarafı her 1 saniyede bir bu değeri okuyup kırmızı süreyi uzatabilir.
"""

import os
# MPS'in desteklemediği işlemleri otomatik olarak CPU'da yapmasını sağlar
os.environ['PYTORCH_ENABLE_MPS_FALLBACK'] = '1'

import cv2
from ultralytics import YOLO
import torch
import serial
import serial.tools.list_ports
import time

# --- 1. AYARLAR VE PARAMETRELER ---
# Tepeden (top-down) videolarda araçlar küçük görünür; bu nedenle conf düşürülür ve imgsz yükseltilir.
CONF_THRESHOLD = 0.25  # Daha düşük eşik: küçük nesneleri kaçırmamak için
IMG_SIZE = 960         # Daha yüksek çözünürlük (mps hızına göre 1280 da denenebilir)

# Model seçimi: küçük model yerine daha güçlü model tercih edilir.
# Dosya yoksa yolo11n.pt ile devam eder.
MODEL_PATH = "yolo12m.pt"

# Küçük araçları görünür kılmak için frame büyütme
UPSCALE_IF_WIDTH_LT = 1280
UPSCALE_FACTOR = 1.5

# Çok küçük kutuları filtrelemek için minimum alan (piksel^2)
MIN_BOX_AREA = 600

# COCO sınıfları: 2=car, 3=motorcycle, 5=bus, 7=truck
VEHICLE_CLASSES = {2, 3, 5, 7}

# --- AYARLAR ---
# Eğer video 720p ise 600-650 idealdir. 
# Eğer video 1080p ise 900-950 seviyelerine çekebilirsiniz.
LINE_Y = 600  # 450'den 600'e çıkarılarak çizgi çok daha aşağıya çekildi.

# Algılama bölgesini de buna göre daraltabilir veya genişletebilirsiniz.
DETECTION_MARGIN = 40

def find_mcp2221_port():
    """MCP2221 USB-UART köprüsünü otomatik bulur (Explorer 8)."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "MCP2221" in (p.description or ""):
            return p.device
    # fallback: first usbmodem
    for p in ports:
        if "usbmodem" in p.device:
            return p.device
    return None

try:
    port = find_mcp2221_port() or '/dev/cu.usbserial-110'
    ser = serial.Serial(port, 9600, timeout=0.1)
    print(f">>> DONANIM MODU: PIC bağlantısı başarılı. Port: {port}")
except Exception as e:
    print(f">>> SİMÜLASYON MODU: Seri port bulunamadı. ({e})")
    ser = None

# --- 2. MODEL VE CİHAZ YAPILANDIRMASI ---
try:
    model = YOLO(MODEL_PATH)
except Exception:
    model = YOLO('yolo11n.pt')
if torch.backends.mps.is_available():
    model.to('mps')
    print(">>> CİHAZ: Apple Silicon GPU (MPS) aktif.")

video_path = "demo.mp4"  # Demo video yolu
cap = cv2.VideoCapture(video_path)

# Sayım için kalıcı araç kimlikleri ve sayaçlar
vehicle_ids = set()
counter = 0

# PIC'e gönderilecek anlık yoğunluk (ekrandaki takip edilen araç sayısı)
current_density = 0

# 1Hz UART gönderim zamanlayıcısı
last_send = time.time()

print(f"Sistem başlatıldı. Güven Eşiği: {CONF_THRESHOLD}")

while cap.isOpened():
    success, frame = cap.read()
    if not success: break

    # Top-down videoda küçük nesneler için frame'i büyüt
    if frame.shape[1] < UPSCALE_IF_WIDTH_LT:
        frame = cv2.resize(
            frame,
            None,
            fx=UPSCALE_FACTOR,
            fy=UPSCALE_FACTOR,
            interpolation=cv2.INTER_CUBIC
        )

    # --- 3. FİLTRELENMİŞ TAKİP (TRACKING) ---
    # conf: Düşük güvenli nesneleri eler
    # iou: Üst üste binen kutuları tek araç olarak sayar
    results = model.track(
        frame, 
        persist=True, 
        classes=list(VEHICLE_CLASSES),  # yalnızca araç sınıfları
        conf=CONF_THRESHOLD,
        iou=0.5,
        imgsz=IMG_SIZE,
        verbose=False
    )

    if results[0].boxes.id is not None:
        boxes = results[0].boxes.xyxy.cpu().numpy()
        ids = results[0].boxes.id.cpu().numpy().astype(int)
        confs = results[0].boxes.conf.cpu().numpy() # Güven oranlarını al
        clss = results[0].boxes.cls.cpu().numpy().astype(int)

        # Anlık yoğunluk: ekrandaki araç sayısı (sadece vehicle classes)
        current_density = int(sum(1 for c in clss if c in VEHICLE_CLASSES))

        for box, id, conf, cls_id in zip(boxes, ids, confs, clss):
            if cls_id not in VEHICLE_CLASSES:
                continue
            x1, y1, x2, y2 = box
            center_y = (y1 + y2) / 2
            box_area = (x2 - x1) * (y2 - y1)
            if box_area < MIN_BOX_AREA:
                continue
            
            # --- 4. AKILLI SAYIM MANTIĞI ---
            # 1. Koşul: Araç daha önce sayılmamış olmalı
            # 2. Koşul: Aracın merkezi sayım çizgisi (LINE_Y) üzerinde olmalı
            # 3. Koşul: Araç belirli bir dikey bölge içinde olmalı (ROI kısıtlaması)
            if id not in vehicle_ids:
                if center_y > LINE_Y and center_y < (LINE_Y + DETECTION_MARGIN):
                    vehicle_ids.add(id)
                    counter += 1
                    print(f"[OK] Araç Sayıldı! ID: {id} | Güven: {conf:.2f} | Toplam: {counter}")
                    
                    # Artık tek tek '1' göndermiyoruz; 1Hz yoğunluk mesajı gönderiyoruz.
                    pass

    # --- 4.5 UART: Her 1 saniyede bir yoğunluk gönder ---
    # Format: C:07\n
    now = time.time()
    if ser and (now - last_send) >= 1.0:
        last_send = now
        value = max(0, min(current_density, 99))
        msg = f"C:{value:02d}\n".encode()
        ser.write(msg)
        # Debug log
        print(f"[UART] sent C:{value:02d}")

    # --- 5. GÖRSELLEŞTİRME ---
    annotated_frame = results[0].plot() 
    
    # --- Görselleştirme (Demo Sunumu İçin) ---
    # 1. Ana Sayım Çizgisi (Kırmızı - Kalın)
    cv2.line(annotated_frame, (0, LINE_Y), (int(frame.shape[1]), LINE_Y), (0, 0, 255), 3)

    # 2. Aktif Algılama Bölgesi Sınırı (Sarı - İnce)
    # Araç bu iki çizgi arasına girdiğinde '1' sinyali tetiklenir.
    cv2.line(annotated_frame, (0, LINE_Y + DETECTION_MARGIN), (int(frame.shape[1]), LINE_Y + DETECTION_MARGIN), (0, 255, 255), 1)

    # 3. Bölgeyi görsel olarak vurgulamak için (Opsiyonel: Şeffaf Katman)
    # Bu kısım hocanıza sunum yaparken "bakın bu bölgeyi tarıyorum" demek için iyidir.
    overlay = annotated_frame.copy()
    cv2.rectangle(overlay, (0, LINE_Y), (int(frame.shape[1]), LINE_Y + DETECTION_MARGIN), (0, 255, 255), -1)
    cv2.addWeighted(overlay, 0.2, annotated_frame, 0.8, 0, annotated_frame)


    cv2.putText(annotated_frame, f"Arac Sayisi: {counter}", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(annotated_frame, f"Güven Esigi: {CONF_THRESHOLD}", (20, 80), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    cv2.imshow("Vision-Enhanced Traffic Controller (YOLO11)", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"): break

cap.release()
cv2.destroyAllWindows()