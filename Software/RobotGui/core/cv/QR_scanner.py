import cv2

def qr_scanner(img):
        scan = cv2.QRCodeDetector()
        coords, box, straight_qrcode = scan.detectAndDecode(img)
        if coords == "":
            return "no QR code"
        parts = coords.split("&")
        try:
            x = float(parts[0].split("=")[1])
            y = float(parts[1].split("=")[1])
        except (IndexError, ValueError):
            return "invalid QR data"
        return f"{x}, {y}"