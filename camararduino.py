import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# Dimensiones de la imagen
w, h = 360, 240

# Rango de área para el seguimiento del rostro
fbRange = [6200, 6800]

# Parámetros del controlador PD
pid = [0.1, 0.2, 0]

# Altura del dron con respecto al suelo
H = 100  # Altura del dron en centímetros

# Altura de la cara con respecto al suelo (por ejemplo, altura promedio de la cabeza humana)
h = 20   # Altura de la cara en centímetros

# Configuración de detección de rostros con MediaPipe
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# Serial communication with Arduino
arduino_port = 'COM5'  # Change this to the appropriate port of your Arduino
baud_rate = 9600
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Allow time for the Arduino to reset

def calculate_depth(width_px):
    # Estimación de la distancia utilizando la relación de semejanza de triángulos
    return (average_face_width_cm * focal_length_px) / width_px

# Distancia focal de la cámara (en píxeles) - Ajustar según la cámara utilizada
focal_length_px = 500

# Anchura promedio del rostro de una persona adulta (en centímetros)
average_face_width_cm = 14

def map_to_xyz(cx, cy):
    # Mapping 2D coordinates (cx, cy) to 3D coordinates (x, y, z)
    x = cx - w // 2  # Mapping image x to world x
    y = cy - h // 2  # Mapping image y to world y

    # Calculating z using similar triangles
    z = (H * h) / y if y != 0 else 0

    return x, y, z

def main():
    pError = 0  # Variable de error anterior
    cx = 0  # Coordenada x del centro del rostro
    cy = 0  # Coordenada y del centro del rostro
    area = 0  # Área del rostro detectado

    cap = cv2.VideoCapture(0)  # Accessing the camera (0 is the default camera index)

    with mp_face_detection.FaceDetection(min_detection_confidence=0.75) as face_detection:
        while True:
            # Captura y procesamiento de la imagen de la cámara
            ret, img = cap.read()
            img = cv2.resize(img, (w, h))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # Detección de rostros
            results = face_detection.process(img)

            # Conversión de vuelta a formato BGR para visualización
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            if results.detections:
                for face_no, detection in enumerate(results.detections):
                    mp_drawing.draw_detection(img, detection)
                    if face_no == 0:
                        xmin = int(detection.location_data.relative_bounding_box.xmin * w)
                        ymin = int(detection.location_data.relative_bounding_box.ymin * h)
                        xd = int(detection.location_data.relative_bounding_box.width * w)
                        yd = int(detection.location_data.relative_bounding_box.height * h)
                        cx, cy, area = (xmin + (xd / 2), ymin + (yd / 2), xd * yd)
                        cv2.putText(img, "Tracking", (xmin, ymin), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

                        # Mapping to x, y, z coordinates
                        x, y, z = map_to_xyz(cx, cy)
                        print("x:", x, "y:", y, "z:", z)

                        # Sending x, y, and z values to Arduino
                        ser.write(f'{x},{y},{z}\n'.encode())

            fb_speed = 0  # Velocidad de avance/retroceso del dron
            error = cx - w // 2  # Error de seguimiento
            yaw_speed = pid[0] * error + pid[1] * (error - pError)  # Velocidad de giro (Yaw)
            yaw_speed = int(np.clip(yaw_speed, -100, 100))  # Clip de la velocidad entre -100 y 100

            # Control de proximidad para ajustar la velocidad de avance/retroceso
            if area > fbRange[0] and area < fbRange[1]:
                fb_speed = 0
            elif area > fbRange[1]:
                fb_speed = -20
            elif area < fbRange[0] and area != 0:
                fb_speed = 20

            # Detener el giro si no se detecta ningún rostro
            if cx == 0:
                yaw_speed = 0
                error = 0

            cv2.imshow("Output", img)

            # Salir del bucle si se presiona la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
