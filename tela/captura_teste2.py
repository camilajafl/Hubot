# captura_teste.py
import cv2
import os
import sys

# recebe nome como argumento
if len(sys.argv) < 2:
    print("Uso: python captura_teste.py <nome>")
    exit(1)

name = sys.argv[1]
folder = "dataset/" + name
if not os.path.exists(folder):
    os.makedirs(folder)

cam = cv2.VideoCapture(0)
cv2.namedWindow("press space to take a photo", cv2.WINDOW_NORMAL)
cv2.resizeWindow("press space to take a photo", 500, 300)

img_counter = 0

while True:
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("press space to take a photo", frame)

    k = cv2.waitKey(1)
    if k % 256 == 27:  # ESC
        break
    elif k % 256 == 32:  # SPACE
        img_name = f"{folder}/image_{img_counter}.jpg"
        cv2.imwrite(img_name, frame)
        print(f"{img_name} written!")
        img_counter += 1

cam.release()
cv2.destroyAllWindows()
