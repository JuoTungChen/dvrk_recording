import cv2

def list_available_ports():
    available_ports = []
    for port in range(0, 11):
        cap = cv2.VideoCapture(port)
        if cap.isOpened():
            available_ports.append(port)
            cap.release()
    return available_ports

available_ports = list_available_ports()
print("Available video ports:", available_ports)
