import math
import sys
import rclpy
import tf_transformations
from rclpy.node import Node   
from rclpy.executors import ExternalShutdownException    
from geometry_msgs.msg import Twist  # Twistメッセージ型をインポート
from nav_msgs.msg import Odometry    # Odometryメッセージ型をインポート
from tf_transformations import euler_from_quaternion 


class HappyMove(Node):  # 簡単な移動クラス
    def __init__(self):   # コンストラクタ
        super().__init__('happy_move_node')        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)   
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0
        self.vel = Twist()  # Twist メッセージ型インスタンスの生成
        self.set_vel(0.0, 0.0)  # 速度の初期化
 
    def get_pose(self, msg):      # 姿勢を取得する
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        return x, y, yaw
  
    def odom_cb(self, msg):         # オドメトリのコールバック関数
        self.x, self.y, self.yaw = self.get_pose(msg)
        self.get_logger().info(
            f'x={self.x: .2f} y={self.y: .2f}[m] yaw={self.yaw: .2f}[rad/s]')     
    
    def set_vel(self, linear, angular):  # 速度を設定する
        self.vel.linear.x = linear   # [m/s]
        self.vel.angular.z = angular  # [rad/s]    
    
    def move_distance(self, dist):  # 指定した距離distを移動する
        error = 0.03  # 許容誤差 [m] 
        diff = dist - math.sqrt((self.x-self.x0)**2 + (self.y-self.y0)**2) 
        if math.fabs(diff) > error:
            self.set_vel(0.35, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True

    def back_distance(self,dist2):
        error = 0.03
        diff = 1 -  (dist2 - math.sqrt((self.x-self.x0)**2 + (self.y-self.y0)**2)) 
        if math.fabs(diff) > error:
            self.set_vel(-0.35, 0.0)
            return False
        else:
            self.set_vel(0.0, 0.0)
            return True


    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ

    def mirror_move(self,distance):
        state = 0
        press_key = 0
        while rclpy.ok():
            if state == 0 and press_key == 0:
                key = input('press e to enter')
                if key == 'e':
                    press_key = 1
                    if self.move_distance(distance):
                      print('complete')
                      state = 1
            elif state == 0 and press_key == 1:
                if self.move_distance(distance):
                      print('complete')
                      state = 1
                      press_key = 0
            elif state == 1 and press_key == 0:
                key = input('press x to exit')
                if key == 'x':
                    press_key = 1
                    if self.back_distance(distance):
                      print('complete')
                      break
            elif state == 1 and press_key == 1:
                if self.back_distance(distance):
                    print('complete')
                    break
            else:
                print('error')
            rclpy.spin_once(self) 
        self.set_vel(0.0, 0.0)
        self.pub.publish(self.vel)



def main(args=None):  # main関数
    rclpy.init(args=args)
    node = HappyMove()

    try:
        node.mirror_move(1.0)
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')     
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
