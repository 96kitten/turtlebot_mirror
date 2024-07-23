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
        super().__init__('mirror_rotate_node')        
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
    

    def rotate_angle(self, angle):  # 指定した角度angleを回転する
        print('now rad = ' + str(self.yaw))
        print('first rad = ' + str(self.yaw0))

        error = 0.01
        diff = self.yaw - self.yaw0

        while diff <= -math.pi:
            diff += 2 * math.pi
        while diff > math.pi:
            diff -= 2 * math.pi
        target_angle = angle


        while target_angle <= -math.pi:
            target_angle += 2 * math.pi
        while target_angle > math.pi:
            target_angle -= 2 * math.pi
        
        angle_error = abs(target_angle - diff)

        print(angle_error)

        if angle == 0:
            angle_error = abs(self.yaw0 - self.yaw)
            target_angle = self.yaw0

        if angle_error >math.pi:
            angle_error = 2 * math.pi - angle_error

        if angle_error > error:
            if target_angle > diff:
                self.set_vel(0.0,0.25)
            else:
                self.set_vel(0.0, -0.25)
            print('rotate continue')
            return False
        else:
            self.set_vel(0.0, 0.0)
            rclpy.spin_once(self)
            print('rotate finish')
            return True


    def timer_callback(self):  # タイマーのコールバック関数
        self.pub.publish(self.vel)  # 速度指令メッセージのパブリッシュ 
       
       
    #def happy_move(self,angle):  # 簡単な状態遷移
     #   state = 0
      #  while rclpy.ok():
       #     if state == 0:
        #        if self.rotate_angle(angle):
        #            print(math.radians(angle))
         #           break
          #  else:
           #     print('エラー状態')
            #rclpy.spin_once(self)

    def happy_move(self):  # 簡単な状態遷移
        while rclpy.ok():
            try:
                rotangle = float(input("input rotate angle"))
                if rotangle == 999 :
                    print('finish!')
                    break
                while not self.rotate_angle(math.radians(rotangle)):
                    rclpy.spin_once(self)
                #rotangle = int(input("input rotate angle"))
            except KeyboardInterrupt:
                break
                


def main(args=None):  # main関数
    rclpy.init(args=args)
    node = HappyMove()

    try:
        #rotangle = int(input("input rotate angle"))
        #node.happy_move(math.radians(rotangle))

        node.happy_move()

        #node.happy_move(int(rotangle))
        #setangle = (2 / math.floor(360 / int(rotangle))) * math.pi
        #node.happy_move(int(setangle))
    except KeyboardInterrupt:
        print('Ctrl+Cが押されました．')     
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
