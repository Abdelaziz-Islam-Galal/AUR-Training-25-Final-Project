import cv2
import numpy as np

def line_detection(img):

    while True:

        # Get frame height and width
        h, w, _ = img.shape

        # Take only the bottom strip (last 50 pixels)
        bottom = img[h-50:h, :]

        # Convert to grayscale
        gray = cv2.cvtColor(bottom, cv2.COLOR_BGR2GRAY)

        # Threshold to detect dark (black) areas
        _, mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

       
        row_sum = np.mean(mask, axis=1)  # average brightness per row
        average_black = np.mean(row_sum)  # average over all bottom rows

        # If most of the bottom is black -> full-width line detected
        if average_black > 200:  # 200 means mostly white in mask (black in image)
            print("STOP – full black line detected!")
            return "stop"
        else:
            print("No line or small black area")

