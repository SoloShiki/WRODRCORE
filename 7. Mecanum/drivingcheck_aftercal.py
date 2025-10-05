#!/usr/bin/env python3
# compare_odom_tf_and_move_calibrated.py
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class CompareOdomTF(Node):
    def __init__(self):
        super().__init__('compare_odom_tf')

        # ---- state ----
        self.odom = None
        self.start_odom = None
        self.start_tf = None
        self.target_distance = 1.0  # meters
        self.speed = 0.2            # m/s
        self.moving = True          # control flag
        #self.odom_scale = 0.64      # apply your calibration factor here
        #self.odom_scale = 0.406      # apply your calibration factor here
        #self.odom_scale = 1.45      # apply your calibration factor here
        self.odom_scale = 1.00      # apply your calibration factor here
        
        #se uso 1.00 for odom_scale, 1.45 hace que el robot avance perfectamente asi
        #que se hizo permanente en el archivo YAML , el anterior era 1.39
        #asi que el nuevo es 1.39 X 1.45 =2.02       


        # ---- subscribers/publishers ----
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')
        navigation_package_path = get_package_share_directory('navigation')
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'

    sim = LaunchConfiguration('sim', default='false').perform(context)
    map_name = LaunchConfiguration('map', default='').perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER']).perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    frame_prefix = '' if robot_name == '/' else '%s/'%robot_name
    topic_prefix = '' if robot_name == '/' else '/%s'%robot_name
    map_frame = '{}map'.format(frame_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)
    depth_camera_topic = '/ascamera/camera_publisher/depth0/image_raw'.format(topic_prefix)
    depth_camera_info = '/ascamera/camera_publisher/rgb0/camera_info'.format(topic_prefix)
    rgb_camera_topic = '/ascamera/camera_publisher/rgb0/image'.format(topic_prefix)
    odom_topic = '{}/odom'.format(topic_prefix)
    scan_topic = '{}/scan_raw'.format(topic_prefix)

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name,
            'action_name': 'horizontal',
        }.items(),
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': os.path.join(slam_package_path, 'maps', map_name + '.yaml'),
            'params_file': os.path.join(navigation_package_path, 'config', 'nav2_params.yaml'),
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'rtabmap': 'true',
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_name),
         base_launch,
         TimerAction(
             period=10.0,  # 延时等待其它节点启动好(delay for enabling other nodes)
             actions=[navigation_launch],
         ),
         rtabmap_launch
      ]
    )

    return [sim_arg, master_name_arg, robot_name_arg, bringup_launch]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint'

        # ---- timers ----
        self.timer = self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("Starting calibrated movement + odom vs TF comparison...")

    def odom_cb(self, msg):
        self.odom = msg

    def lookup_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return trans
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.moving = False
        self.get_logger().info("✅ Stopped robot at target distance")

    def timer_cb(self):
        if self.odom is None:
            return

        tf_trans = self.lookup_tf()

        # capture starting positions
        if self.start_odom is None and tf_trans is not None:
            self.start_odom = (
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y
            )
            self.start_tf = (
                tf_trans.transform.translation.x,
                tf_trans.transform.translation.y
            )
            self.get_logger().info("Start positions captured.")
            return

        if self.start_odom is None:
            return

        # odom distance (scaled)
        ox = self.odom.pose.pose.position.x - self.start_odom[0]
        oy = self.odom.pose.pose.position.y - self.start_odom[1]
        odom_dist = math.sqrt(ox * ox + oy * oy) * self.odom_scale

        # tf distance (unscaled)
        tf_dist = None
        if tf_trans is not None and self.start_tf is not None:
            tx = tf_trans.transform.translation.x - self.start_tf[0]
            ty = tf_trans.transform.translation.y - self.start_tf[1]
            tf_dist = math.sqrt(tx * tx + ty * ty)

        # logging
        if tf_dist is None:
            self.get_logger().info(f"odom (scaled): {odom_dist:.3f} m | tf: (no tf yet)")
        else:
            self.get_logger().info(f"odom (scaled): {odom_dist:.3f} m | tf: {tf_dist:.3f} m")

        # movement logic
        if self.moving:
            if odom_dist < self.target_distance:
                twist = Twist()
                twist.linear.x = self.speed
                self.cmd_pub.publish(twist)
            else:
                self.stop_robot()


def main():
    rclpy.init()
    node = CompareOdomTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
