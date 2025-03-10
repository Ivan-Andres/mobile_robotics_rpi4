import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,MagneticField

from rclpy.clock import Clock
import math
import numpy as np

from Rosmaster_Lib import Rosmaster

class motion(Node):
    def __init__(self):
        super().__init__('motion')
        print("Node started")

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.imuPublisher = self.create_publisher(Imu,"/imu/data_raw",10)
        self.magPublisher = self.create_publisher(MagneticField,"/imu/mag",10)
        self.volPublisher = self.create_publisher(Float32,"voltage",10)

        #robot Create the Rosmaster object bot
        self.robot = Rosmaster()
        self.robot.create_receive_threading()

        self.throttle = 90
        self.steering = 90

        self.prev_time = None
        self.vx = 0.0
        self.vy = 0.0

        # Variables de aceleración filtrada
        self.ax_filtered = 0.0
        self.ay_filtered = 0.0
        self.az_filtered = 0.0

        self.ax_v_filtered = 0.0
        self.ay_v_filtered = 0.0
        self.az_v_filtered = 0.0

        self.ax_v = 0.0
        self.ay_v = 0.0
        self.az_v= 0.0

        # Coeficiente del filtro pasabajas (ajustar según el ruido del sensor)
        self.alpha = 0.1  # Valores bajos filtran más


    def cmd_vel_callback(self, msg):
        time_stamp = Clock().now()
        imu = Imu()
        mag = MagneticField()
        battery = Float32()
        eje_velocidad = msg.linear.x
        eje_direccion = msg.angular.z

        battery.data = self.robot.get_battery_voltage()
        ax, ay, az = self.robot.get_accelerometer_data()
        gx, gy, gz = self.robot.get_gyroscope_data()
        mx, my, mz = self.robot.get_magnetometer_data()
        mx = mx * 1.0
        my = my * 1.0
        mz = mz * 1.0

        # Aplicar filtro pasabajas
        self.ax_filtered = self.alpha * ax + (1 - self.alpha) * self.ax_filtered
        self.ay_filtered = self.alpha * ay + (1 - self.alpha) * self.ay_filtered
        self.az_filtered = self.alpha * az + (1 - self.alpha) * self.az_filtered

        #self.get_logger().info(f"ay: {self.ay_filtered:.6f}, ax: {self.ax_filtered:.6f}, az: {self.az_filtered:.6f}")

        roll, pitch, yaw = self.robot.get_imu_attitude_data()
        roll = -(roll + 1.08)
        pitch = -(pitch + 1.34) 
        yaw = -yaw
        #self.get_logger().info(f"roll: {roll:.6f}, pitch: {pitch:.6f}, yaw: {yaw:.6f}")

        self.ax_v = ((self.ax_filtered) + (9.83 * math.cos(math.radians((-90)-roll))))-0.2
        self.ay_v = ((self.ay_filtered) + (9.83 * math.cos(math.radians(90-pitch))))+0.23
        self.az_v = ((self.az_filtered) + (9.83 * math.sin(math.radians(90-pitch))))+0.02

        # Aplicar filtro pasabajas a ax_v, ay_v y az_v
        self.ax_v_filtered = self.alpha * self.ax_v + (1 - self.alpha) * self.ax_v_filtered
        self.ay_v_filtered = self.alpha * self.ay_v + (1 - self.alpha) * self.ay_v_filtered
        self.az_v_filtered = self.alpha * self.az_v + (1 - self.alpha) * self.az_v_filtered

        now = self.get_clock().now().to_msg()
        current_time = now.sec + (now.nanosec * 1e-9)

        if self.prev_time is not None:
            dt = current_time - self.prev_time
            self.vx += (self.ax_v_filtered) * dt
            self.vy += (self.ay_v_filtered) * dt

        self.get_logger().info(f"velocidad lineal en y: {self.vy}, velocidad lineal en x: {self.vx}")

        self.prev_time = current_time

        # Publish gyroscope data
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = self.ax_v_filtered
        imu.linear_acceleration.y = self.ay_v_filtered
        imu.linear_acceleration.z = self.az_v_filtered
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = 'imu_link'
        mag.magnetic_field.x = mx
        mag.magnetic_field.y = my
        mag.magnetic_field.z = mz

        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.volPublisher.publish(battery)

        self.get_logger().info(f'Velocidad: {eje_velocidad},direccion: {eje_direccion}')
        self.throttle = self.map(eje_velocidad, -1.0, +1.0, 70, 110)
        self.steering = self.map(eje_direccion, -1.0, +1.0, 60.0, 120.0)

        self.get_logger().info(f'thtottle: {self.throttle},steering: {self.steering}')
        self.robot.set_pwm_servo(4,self.throttle)
        self.robot.set_pwm_servo(3,self.steering)

    def map(self,x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def kill(self):
        del self.robot

def main(args=None):
    rclpy.init(args=args)
    node = motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
