import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections_0"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"
#----------------------------------------------

TIMER = 0.1

class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.integral_limit = integral_limit

    def calculate(self, error, dt):
        if dt == 0:
            return 0
        
        self.integral += error * dt
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
            
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.timer_period = self.declare_parameter('timer', TIMER).value
        
        self.steering_pid = PIDController(kp=0.2, ki=0.005, kd=0.01) 
        self.last_time = self.get_clock().now()
        self.vehicle_position = (300,470)
        self.image_center = self.declare_parameter('image_center', 300).value

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None
        self.steering_command = 0.0
        self.left_speed_command = 0
        self.right_speed_command = 0
        
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Motion Planner Node has been started.')

    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

    def path_callback(self, msg: PathPlanningResult):
        self.get_logger().info('<<<<< Path data received! >>>>>')
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if self.lidar_data is not None and self.lidar_data.data is True:
            self.steering_command = 0.0 
            self.left_speed_command = 0 
            self.right_speed_command = 0 
        elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red' and self.detection_data is not None:
             for detection in self.detection_data.detections:
                if detection.class_name=='traffic_light':
                    y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2)
                    if y_max < 150:
                        self.steering_command = 0.0 
                        self.left_speed_command = 0 
                        self.right_speed_command = 0
        else:
            if self.path_data is None or len(self.path_data) == 0:
                self.steering_command = 0.0
                self.left_speed_command = 0
                self.right_speed_command = 0
            else:
                try:
                    # [핵심 수정] ROI 크기에 무관한 동적 목표 지점 설정
                    path_len = len(self.path_data)
                    
                    # 가까운 목표점: 경로의 70% 지점 (인덱스는 0부터 시작하므로 path_len * 0.3)
                    near_target_point = self.path_data[int(path_len * 0.3)]
                    
                    # 먼 목표점: 경로의 30% 지점 (인덱스는 0부터 시작하므로 path_len * 0.7)
                    far_target_point = self.path_data[int(path_len * 0.7)]
                    
                    # 두 목표점의 x 좌표에 가중 평균을 주어 최종 목표 x 좌표 계산
                    # 먼 곳의 경로를 더 중요하게 생각하여 가중치를 0.6으로 설정
                    weighted_target_x = (near_target_point[0] * 0.4) + (far_target_point[0] * 0.6)

                    cross_track_error = weighted_target_x - self.image_center
                    
                    steering_adjustment = self.steering_pid.calculate(cross_track_error, dt)

                    max_steering = 7.0
                    self.steering_command = max(-max_steering, min(steering_adjustment, max_steering))

                    self.get_logger().info(f"Target X: {weighted_target_x:.2f}, CTE: {cross_track_error:.2f}, Steering Cmd: {self.steering_command:.2f}")

                    self.left_speed_command = 100
                    self.right_speed_command = 100

                except IndexError:
                    self.get_logger().warn("Path data is not long enough. Stopping.")
                    self.steering_command = 0.0
                    self.left_speed_command = 0
                    self.right_speed_command = 0

        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command 
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()