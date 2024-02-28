import cv2
import mediapipe as mp
import numpy as np

# Dimensiones de la imagen
w, h = 360, 240

# Anchura promedio del rostro de una persona adulta (en centímetros)
average_face_width_cm = 14

# Distancia focal de la cámara (en píxeles) - Ajustar según la cámara utilizada
focal_length_px = 500

# Rango de área para el seguimiento del rostro
fbRange = [6200, 6800]

# Parámetros del controlador PD
pid = [0.1, 0.2, 0]

# Configuración de detección de rostros con MediaPipe
mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

def calculate_depth(width_px):
    # Estimación de la distancia utilizando la relación de semejanza de triángulos
    return (average_face_width_cm * focal_length_px) / width_px

def main():
    pError = 0  # Variable de error anterior
    cx = 0  # Coordenada x del centro del rostro
    area = 0  # Área del rostro detectado

    cap = cv2.VideoCapture(0)  # Accessing the camera (0 is the default camera index)

    
    cap.set(3, w)  # Set width
    cap.set(4, h)  # Set height

    with mp_face_detection.FaceDetection(min_detection_confidence=0.75) as face_detection:
        while True:
            # Captura y procesamiento de la imagen de la cámara
            ret, img = cap.read()

            
            if not ret:
                print("Error reading frame from camera.")
                continue  # exit the loop if reading fails

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

                        # Estimación de la profundidad (z)
                        depth_cm = calculate_depth(xd)
                        z = depth_cm

                        # Mapeo de las coordenadas x, y, y z
                        x = (cx - w // 2) * (depth_cm / focal_length_px)
                        y = (cy - h // 2) * (depth_cm / focal_length_px)

                        print("Face coordinates (x, y, z): ({:.2f}, {:.2f}, {:.2f} cm)".format(x, y, z))

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
