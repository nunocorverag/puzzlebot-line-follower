#!/usr/bin/env python3
"""
Behavior Controller (Iteración 1 - Modo Copiloto)

Escucha los tópicos de visión y calcula qué debería hacer el robot.
Implementa jerarquía de prioridades y memoria a corto plazo (cooldowns).
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BehaviorController(Node):
    def __init__(self):
        super().__init__("behavior_controller")

        # --- Suscriptores ---
        self.create_subscription(String, "/sign_detection", self.sign_callback, 10)
        self.create_subscription(String, "/traffic_light_state", self.light_callback, 10)

        # --- Estado del Mundo ---
        self.current_light = "NONE"
        self.last_seen_sign = "none"
        
        # --- Variables de Control (Memoria) ---
        self.current_state = "CRUISE"  # CRUISE, STOPPING, WAITING, TURNING
        self.ignore_signs_until = 0.0  # Tiempo (epoch) para el Cooldown
        self.cooldown_duration = 4.0   # Segundos para ignorar letreros después de procesar uno

        # --- Loop de Decisión (10 Hz) ---
        self.create_timer(0.1, self.decision_loop)
        
        self.get_logger().info("Cerebro iniciado en MODO COPILOTO. Esperando datos...")

    def light_callback(self, msg):
        self.current_light = msg.data

    def sign_callback(self, msg):
        # Solo guardamos el letrero si NO estamos en cooldown
        if time.time() > self.ignore_signs_until:
            self.last_seen_sign = msg.data

    def decision_loop(self):
        now = time.time()
        proposed_action = "Avanzar normal"

        # ==========================================
        # JERARQUÍA DE PRIORIDADES
        # ==========================================

        # 1. PRIORIDAD ABSOLUTA: Semáforos (Sobrescribe todo)
        if self.current_light == "RED":
            self.current_state = "WAITING"
            proposed_action = "ALTO TOTAL (Semáforo Rojo)"
        
        elif self.current_light == "YELLOW":
            self.current_state = "STOPPING"
            proposed_action = "FRENANDO (Semáforo Amarillo)"

        # 2. PRIORIDAD ALTA: Letreros de Control (Stop / Give Way)
        elif self.last_seen_sign in ["stop", "give-way"]:
            self.current_state = "STOPPING"
            proposed_action = f"ALTO (Vio letrero: {self.last_seen_sign.upper()})"
            
            # Simulamos que ya procesó el alto, aplicamos cooldown para olvidarlo
            self.ignore_signs_until = now + self.cooldown_duration
            self.last_seen_sign = "none" # Lo borramos de la memoria

        # 3. PRIORIDAD MEDIA: Letreros de Navegación
        elif self.last_seen_sign in ["vuelta-derecha", "vuelta-izquierda", "straight"]:
            self.current_state = "TURNING"
            proposed_action = f"PREPARANDO GIRO ({self.last_seen_sign.upper()})"
            
            # Aplicamos cooldown para no leer la misma flecha mientras da la vuelta
            self.ignore_signs_until = now + self.cooldown_duration
            self.last_seen_sign = "none"

        # 4. PRIORIDAD BAJA: Precaución
        elif self.last_seen_sign == "trabajadores":
            self.current_state = "CRUISE_SLOW"
            proposed_action = "REDUCIENDO VELOCIDAD (Zona de obras)"
            self.ignore_signs_until = now + self.cooldown_duration
            self.last_seen_sign = "none"

        # 5. SIN ESTÍMULOS (Luz verde o nada en el horizonte)
        else:
            self.current_state = "CRUISE"
            if self.current_light == "GREEN":
                proposed_action = "Avanzar (Semáforo Verde)"
            elif now < self.ignore_signs_until:
                proposed_action = f"Avanzar (Ignorando letreros por {self.ignore_signs_until - now:.1f}s)"
            else:
                proposed_action = "Avanzar (Sigue la línea)"

        # --- Consola de Debug (Se actualiza en la misma línea para no hacer spam) ---
        print(f"\rESTADO: [{self.current_state:^12}] | ACCIÓN: {proposed_action:<40}", end="", flush=True)

def main():
    rclpy.init()
    node = BehaviorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nApagando cerebro...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()