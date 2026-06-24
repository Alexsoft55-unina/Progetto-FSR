import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import math

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        
        # Ci abboniamo ai dati dell'IMU
        self.sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Creiamo il megafono per comandare le ruote
        self.pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        
        # ==========================================
        # I MAGICI PARAMETRI PID (Da tarare!)
        # ==========================================
        # Kp: Se cadiamo in avanti, quanto forte spingiamo le ruote in avanti?
        self.kp = 15.0 
        
        # Kd: Quanto cerchiamo di frenare il movimento se stiamo cadendo troppo veloci?
        self.kd = 2.0  
        
        # L'angolo desiderato (0.0 = dritto in piedi)
        self.target_pitch = 0.0

    def imu_callback(self, msg):
        # 1. Calcoliamo di quanto siamo inclinati (Pitch) a partire dal Quaternione
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi/2, sinp)

        # 2. Quanto stiamo sbagliando rispetto allo stare dritti?
        error = self.target_pitch - pitch
        
        # 3. A che velocità stiamo cadendo in questo istante? (Giroscopio asse Y)
        pitch_rate = msg.angular_velocity.y
        
        # 4. LA FORMULA DEL CONTROLLORE PD
        # Sommiamo la forza proporzionale e quella derivativa
        # (Nota: Il segno dipende da come hai orientato gli assi, potresti doverlo invertire in -kp)
        output = (self.kp * error) - (self.kd * pitch_rate)        
        # 5. Non mandiamo velocità impossibili ai motori (max 20 rad/s)
        max_speed = 20.0
        output = max(min(output, max_speed), -max_speed)
        
        # 6. Invia il comando alle due ruote!
        out_msg = Float64MultiArray()
        # Se il robot gira su se stesso invece di andare dritto, uno di questi due output va invertito (-output)
        out_msg.data = [output, output]
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    print("Controllore di Equilibrio AVVIATO! Che la forza sia con noi...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()