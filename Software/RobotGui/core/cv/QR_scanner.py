import cv2
import re

def qr_scanner(img):
        scan = cv2.QRCodeDetector()
        coords, box, straight_qrcode = scan.detectAndDecode(img)
        if coords == "":
            return "no QR code"
        parts = re.findall(r"\d+\.?\d*", coords)

        try:
            x = float(parts[0])
            y = float(parts[1])
        except (IndexError, ValueError):
            return coords
        return f"{x}, {y}"
