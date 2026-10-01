import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import math
from sensor_msgs.msg import Imu, MagneticField
# Import các Message chuẩn của ROS 2
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class Stm32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')
        
        # --- Khai báo Parameters Serial ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        # --- THÔNG SỐ ROBOT (KINEMATICS) ---
        self.declare_parameter('track_width', 0.2)       # Khoảng cách giữa 2 bánh xe (mét)
        self.declare_parameter('ticks_per_meter', 1000.0) # Số xung encoder khi xe đi được 1 mét
        self.declare_parameter('pid_rate', 50.0)          # Tần số PID (đã set 50Hz trong main.cpp)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Đã kết nối STM32 tại {port} với baudrate {baud}")
        except Exception as e:
            self.get_logger().error(f"Lỗi mở cổng Serial: {e}")
            exit()

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
# ĐỔI TÊN TOPIC: Đăng tải lên /imu/data_raw thay vì /imu/data
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10) # THÊM DÒNG NÀY
        self.odom_pub = self.create_publisher(Odometry, '/odom/unfiltered', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x    # m/s
        w = msg.angular.z   # rad/s
        
        track_width = self.get_parameter('track_width').value
        ticks_per_meter = self.get_parameter('ticks_per_meter').value
        pid_rate = self.get_parameter('pid_rate').value
        
        v_left = v - (w * track_width / 2.0)
        v_right = v + (w * track_width / 2.0)
        
        ticks_left = (v_left * ticks_per_meter) / pid_rate
        ticks_right = (v_right * ticks_per_meter) / pid_rate
        
        payload = struct.pack('<ff', ticks_left, ticks_right)
        
        header = bytes([0xA5, 0x5A])
        msg_id = bytes([0x01])  # MSG_CMD_VEL
        length = bytes([len(payload)])
        checksum = bytes([sum(payload) % 256])
        
        packet = header + msg_id + length + payload + checksum
        self.ser.write(packet)

    def serial_read_loop(self):
        state = 0
        payload_len = 0
        msg_id = 0
        payload = bytearray()
        
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                c = self.ser.read(1)[0]
                
                # State Machine
                if state == 0:
                    if c == 0xA5: state = 1
                elif state == 1:
                    if c == 0x5A: state = 2
                    else: state = 0
                elif state == 2:
                    msg_id = c
                    state = 3
                elif state == 3:
                    payload_len = c
                    payload = bytearray()
                    state = 4
                elif state == 4:
                    payload.append(c)
                    if len(payload) >= payload_len:
                        state = 5
                elif state == 5:
                    calc_checksum = sum(payload) % 256
                    if c == calc_checksum:
                        self.process_packet(msg_id, payload)
                    else:
                        self.get_logger().warn("Sai Checksum từ STM32!")
                    state = 0

    def process_packet(self, msg_id, payload):
        current_time = self.get_clock().now().to_msg()
        
        if msg_id == 0x02:  # MSG_IMU_DATA (9 floats = 36 bytes)
            if len(payload) != 36:
                return

            data = struct.unpack('<fffffffff', payload)
            
            imu_msg = Imu()
            imu_msg.header.stamp = current_time
            imu_msg.header.frame_id = 'imu_link'
            
            # Gia tốc (m/s^2)
            imu_msg.linear_acceleration.x = data[0]
            imu_msg.linear_acceleration.y = data[1]
            imu_msg.linear_acceleration.z = data[2]
            
            # Vận tốc góc (rad/s)
            imu_msg.angular_velocity.x = data[3]
            imu_msg.angular_velocity.y = data[4]
            imu_msg.angular_velocity.z = data[5]
            
            # ĐÁNH DẤU LÀ IMU RAW (Không có Quaternion) theo REP-145
            imu_msg.orientation_covariance[0] = -1.0
            
            # Khởi tạo ma trận Covariance (Giá trị mẫu, cần tuning dựa trên EKF)
            imu_msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04
            ]
            imu_msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02
            ]
            
            self.imu_pub.publish(imu_msg)
            # --- BỔ SUNG XỬ LÝ DỮ LIỆU TỪ TRƯỜNG (MAG) ---
            mag_msg = MagneticField()
            mag_msg.header.stamp = current_time
            mag_msg.header.frame_id = 'imu_link'
            
            # Đơn vị chuẩn của ROS cho Mag là Tesla (T)
            # Theo file gy85_driver.h, HMC5883L trả về milliGauss (mG)
            # Quy đổi: 1 mG = 1e-7 Tesla
            mag_msg.magnetic_field.x = data[6] * 1e-7
            mag_msg.magnetic_field.y = data[7] * 1e-7
            mag_msg.magnetic_field.z = data[8] * 1e-7
            
            self.mag_pub.publish(mag_msg)
            
        elif msg_id == 0x03:  # MSG_ODOM_DATA (5 floats = 20 bytes)
            if len(payload) != 20:
                self.get_logger().warn(f"Lỗi độ dài gói ODOM! Kỳ vọng 20, nhận được {len(payload)}")
                return
                
            # Đổi từ 9 float ('<fffffffff') thành 5 float ('<fffff')
            data = struct.unpack('<fffff', payload)
            
            x, y, theta = data[0], data[1], data[2]
            vx, vth = data[3], data[4]
            
            # PC TỰ TÍNH QUATERNION: Dùng Euler to Quaternion cho góc Yaw (theta)
            q0 = math.cos(theta / 2.0)  # w
            q1 = 0.0                    # x
            q2 = 0.0                    # y
            q3 = math.sin(theta / 2.0)  # z
            
            # 1. Publish Transform (TF)
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.w = q0
            t.transform.rotation.x = q1
            t.transform.rotation.y = q2
            t.transform.rotation.z = q3
            
            self.tf_broadcaster.sendTransform(t)
            
            # 2. Publish Odometry Message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            
            odom_msg.pose.pose.orientation.w = q0
            odom_msg.pose.pose.orientation.x = q1
            odom_msg.pose.pose.orientation.y = q2
            odom_msg.pose.pose.orientation.z = q3
            
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.angular.z = vth

            # BỔ SUNG: Ma trận Covariance cho Odometry (Rất quan trọng với robot_localization)
            odom_msg.pose.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,  # Z không đo lường được
                0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,  # Roll
                0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,  # Pitch
                0.0, 0.0, 0.0, 0.0, 0.0, 0.03        # Yaw có độ tin cậy vừa phải
            ]
            odom_msg.twist.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.03
            ]
            
            self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()