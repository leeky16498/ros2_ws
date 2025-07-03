#!/usr/bin/env python3

import math # square root 적용을 위한 라이브러리
import rclpy # Robot Client Library for Python
from functools import partial # 콜백함수에 인자를 고정해 넘기는 용도
from rclpy.node import Node # 노드 클래스
from turtlesim.msg import Pose # turtlesim topic 중 Pose
from geometry_msgs.msg import Twist # geometry_msgs 중 Twist
from my_robot_interfaces.msg import Turtle # 커스텀 인터페이스 디자인
from my_robot_interfaces.msg import TurtleArray # 커스텀 인터페이스 디자인
from my_robot_interfaces.srv import CatchTurtle # 커스텀 인터페이스 디자인
from turtlesim.srv import Kill # turtlesim 서비스 메세지 중 Kill
 
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller") # 노드 이름 설정 / ros2 node list 할 때 이 이름으로 조회, executable name 과는 다름
        self.declare_parameter("catch_closest_turtle_first", True)
        # 파라미터를 입력 받음
        self.turtle_to_catch_ : Turtle = None
        # 잡아야 하는 터틀 객체
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value
        # 터틀 생성 순서로 잡을지 아니면, 가까운 애를 먼저 잡을지 설정 가능

        self.pose_ : Pose= None 
        # 마스터 터틀의 현 위치(x, y, theta, linear_velocity, angular_velocity)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) 
        # Twist라는 메세지 데이터 타입 형태로, 토픽을 발생시킨다. 이름은 "/turtle1/cmd_vel"
        # turtlesim에서는 "turtle1/cmd_vel" 토픽으러 마스터 터틀을 움직일 수 있음
        # 마지막은queue size로 메세지를 버퍼에 얼마자 저장할지를 결정, 네트워크 느릴 시 중요. 퍼블리셔가 섭스크라이버보다 빠르면 이 큐사이즈만큼 저장하고 넘치는 건 버림
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)
        # /turtle1/pose를 구독해 업데이트 하고 새로운 메세지가 올 때마다, callback_pose가 실행
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        # 0.01초 간격으로 control_loop를 자동호출하는 타이머 생성
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        # turtle_spawner에서 생성한 alive_turtles 데이터 리스트를 수신하는 구독자 객체 생성
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")
        # 클라이언트는 특정 서비스 서버와 통신하게 하는 것. CatchTurtle은 서비스 메세지 타입, "catch_turtle"은 호출하려는 서비스 고유 명칭

    def callback_pose(self, pose: Pose):
        self.pose_ = pose

## 가장 가까운 거북이부터 잡아가는 알고리즘
    def callback_alive_turtles(self, msg: TurtleArray):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance

                self.turtle_to_catch_ = closest_turtle

            else:
                self.turtle_to_catch_ = msg.turtles[0]
        
    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return
        
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x*dist_x + dist_y*dist_y)

        cmd = Twist()
        
        if distance > 0.5:
            # position
            cmd.linear.x = 2 * distance

            #orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta

            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < - math.pi:
                diff += 2*math.pi

            cmd.angular.z = 6*diff

        else:
            #target reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.call_catch_turtle_service(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.cmd_vel_publisher_.publish(cmd)

## 목표 터틀에 도달하여 해당 터틀을 잡는 서비스 호출
    def call_catch_turtle_service(self, turtle_name):
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for catch turtle service...")
        ## 클라이언트가 서비스 요청을 하면서 서버가 뜰 때까지 메세지를 출력하며 대기     

        request = CatchTurtle.Request()
        request.name = turtle_name
        ## 요청 메세지 생성

        future = self.catch_turtle_client_.call_async(request)
        ## 비동기식 서비스 요청 전송

        future.add_done_callback(
            partial(self.callback_call_catch_turtle_service, turtle_name=turtle_name)
        )
        # add_done_callback은 서비스 응답이 도착했을 때 실행할 콜백함수 명시
        # partial은 콜백함수에 추가 인자를 넘길 수 있게 한다.
        # callback_ 여기가 실제 콜백함수이며, ROS2는 이 콜백에 future 1개만 넘긴다.하지만 이와 더불어 추가로 넘기고 싶은 인자를 지정했는데 그게 turtle_name

    def callback_call_catch_turtle_service(self, future, turtle_name):

        response: CatchTurtle.Response = future.result()
        if not response.success:
            self.get_logger().error("Turtle " + turtle_name + " could not be removed...!")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()