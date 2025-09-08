import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import PathPlanningResult, MotionCommand # MotionCommand 추가
import cv2
import numpy as np
from cv_bridge import CvBridge

#---------------Variable Setting---------------
SUB_ROI_IMAGE_TOPIC = "roi_image"
SUB_SPLINE_PATH_TOPIC = "path_planning_result"
SUB_STEER_TOPIC = "topic_control_signal"  # motion_planner_node가 발행하는 토픽
PUB_TOPIC_NAME = "path_visualized_img"

#----------------------------------------------
class PathVisualizerNode(Node):
    def __init__(self):
        super().__init__('path_visualizer_node')

        # 파라미터 선언
        self.sub_roi_image_topic = self.declare_parameter('sub_roi_image_topic', SUB_ROI_IMAGE_TOPIC).value
        self.sub_spline_path_topic = self.declare_parameter('sub_spline_path_topic', SUB_SPLINE_PATH_TOPIC).value
        self.sub_steer_topic = self.declare_parameter('sub_steer_topic', SUB_STEER_TOPIC).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value

        # QoS 설정 (가장 먼저 정의)
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # CvBridge 및 변수 초기화
        self.cv_bridge = CvBridge()
        self.roi_image = None
        self.spline_path = None
        self.steering_angle = 0.0 # 조향각 저장 변수

        # 구독자 설정
        self.roi_image_sub = self.create_subscription(
            Image, self.sub_roi_image_topic, self.roi_image_callback, self.qos_profile)
        
        self.spline_path_sub = self.create_subscription(
            PathPlanningResult, self.sub_spline_path_topic, self.spline_path_callback, self.qos_profile)
        
        # MotionCommand 타입으로 수정하고 steer_callback 함수 연결
        self.steer_sub = self.create_subscription(
            MotionCommand, self.sub_steer_topic, self.steer_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(Image, self.pub_topic, self.qos_profile)

        self.get_logger().info("Path Visualizer node has been started.")

    # steer_callback 함수 정의
    def steer_callback(self, msg: MotionCommand):
        self.steering_angle = float(msg.steering) # MotionCommand 메시지에서 steering 값 사용
        # 이미지가 있다면 바로 화면 업데이트
        if self.roi_image is not None:
             self.visualize_path()

    def roi_image_callback(self, msg: Image):
        try:
            self.roi_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 새 이미지를 받으면 바로 화면 업데이트
            self.visualize_path()
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROI image: {str(e)}")

    def spline_path_callback(self, msg: PathPlanningResult):
        self.spline_path = list(zip(msg.x_points, msg.y_points))
        # 새 경로를 받으면 바로 화면 업데이트
        if self.roi_image is not None:
            self.visualize_path()

    def visualize_path(self):
        # roi_image가 없을 경우 아무것도 하지 않음
        if self.roi_image is None:
            return

        # 원본 이미지를 복사하여 사용 (매번 새로 그림)
        vis_image = self.roi_image.copy()

        # 경로 점들을 이미지 위에 그리기
        if self.spline_path is not None:
            for (x, y) in self.spline_path:
                cv2.circle(vis_image, (int(x), int(y)), 3, (0, 0, 255), -1)

        # 조향각 텍스트를 이미지에 추가
        steer_text = f"Steering: {self.steering_angle:.2f}"
        cv2.putText(vis_image, steer_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        # 시각화된 이미지를 ROS 메시지로 변환하여 퍼블리시
        try:
            output_msg = self.cv_bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            self.publisher.publish(output_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image for publishing: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()