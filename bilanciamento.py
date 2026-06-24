import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        self.sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)

        # --- Guadagni PID ---
        # Se il robot oscilla troppo: abbassa kp
        # Se cade prima di reagire: aumenta kp
        # Se deriva lentamente in una direzione: aumenta ki
        # Se oscilla velocemente: aumenta kd
        self.kp = 1000.0   # era 80 — il robot cade in 2s, serve più reazione
        self.ki = 12.0     # abbassa ki per ora, non serve accumulo se cade subito
        self.kd = 2.0    # era 5 — serve più smorzamento sulla velocità angolare

        # Offset: se il robot cade sempre avanti a regime, aumenta (es. 0.03)
        # Se cade sempre indietro, diminuisci (es. -0.03)
        self.target_pitch = 0.00

        # Stato integrale
        self.integral = 0.0
        self.integral_max = 2.0  # anti-windup

        self.last_time = None
        self._count = 0

    def imu_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Calcola pitch dall'orientazione quaternione
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        error = self.target_pitch - pitch
        pitch_rate = msg.angular_velocity.y

        # dt per l'integrale
        if self.last_time is None:
            dt = 0.01
        else:
            dt = now - self.last_time
            dt = max(0.001, min(dt, 0.1))  # clamp: evita dt anomali
        self.last_time = now

        # Integrale con anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))

        # Calcolo PID
        # Segno negativo: se pitch > 0 (robot inclinato avanti) → ruote avanti per recuperare
        output = -(self.kp * error) - (self.ki * self.integral) - (self.kd * pitch_rate)

        # Saturazione
        max_speed = 5.0
        output = max(-max_speed, min(max_speed, output))

        self._count += 1
        if self._count % 100 == 0:
            self.get_logger().info(
                f'pitch={math.degrees(pitch):+.1f}° | '
                f'rate={pitch_rate:+.3f} | '
                f'integral={self.integral:+.3f} | '
                f'output={output:+.2f}'
            )

        out_msg = Float64MultiArray()
        out_msg.data = [output, output]
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    print("Controllore di Equilibrio AVVIATO!")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()