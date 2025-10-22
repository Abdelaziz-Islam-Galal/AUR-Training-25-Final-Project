import cv

def upper_part(img):
    height, width, _ = img.shape

    # Take only the top half of the frame
    upper_part = img[:height // 2, :]