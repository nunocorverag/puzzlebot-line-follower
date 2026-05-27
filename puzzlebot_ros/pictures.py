#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import os

class CalibrationCaptureNode(Node):

    def __init__(self):
        super().__init__('calibration_capture_node')

        # =========================
        # Configuración de Directorio
        # =========================
        self.save_dir = "calibration_images"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Directorio creado: ./{self.save_dir}/")

        # =========================
        # Inicialización de Cámara
        # =========================
        # Utilizamos la misma lógica de GStreamer de tu nodo original
        self.cap = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=640, height=480, framerate=30/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink max-buffers=1 drop=true",
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara.")
            return

        self.get_logger().info("Cámara inicializada correctamente.")
        
        # =========================
        # Parámetros de Captura
        # =========================
        self.image_count = 0
        self.target_count = 75  # Número de fotos a tomar
        
        # Un temporizador que se ejecuta cada 3.0 segundos
        self.timer_period = 0.15 
        self.get_logger().info(f"Iniciando captura automática: 1 foto cada {self.timer_period} segundos.")
        self.timer = self.create_timer(self.timer_period, self.capture_loop)

    def capture_loop(self):
        # Condición de salida al llegar a 75 fotos
        if self.image_count >= self.target_count:
            self.get_logger().info(f"¡Listo! Se han guardado {self.target_count} imágenes en ./{self.save_dir}/")
            self.timer.cancel()
            rclpy.shutdown()
            return

        # Leer frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se recibió ningún frame de la cámara.")
            return

        # Guardar la imagen
        filename = os.path.join(self.save_dir, f"calib_img_{self.image_count:03d}.png")
        cv2.imwrite(filename, frame)
        
        self.image_count += 1
        self.get_logger().info(f"Guardada: {filename} ({self.image_count}/{self.target_count})")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Captura interrumpida por el usuario.")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()