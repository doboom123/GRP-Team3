import cv2

def capture_grayscale():
    camera = cv2.VideoCapture(0)
    ret, image = camera.read()
    camera.release()

    if not ret:
        print("Failed to capture image")
        return None

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    cv2.imshow('Grayscale Image', gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return gray

capture_grayscale()