import cv2
import re

def qr_scanner(img):
        scan = cv2.QRCodeDetector()
        coords, bbox, straight_qrcode = scan.detectAndDecode(img)
        if bbox is None or coords =="":
            return 'no QR code'
            
        parts = re.findall(r"\d+\.?\d*", coords)

        try:
            x = float(parts[0])
            y = float(parts[1])
            return f"{x}, {y}"
        except (IndexError, ValueError):
            return coords
        
