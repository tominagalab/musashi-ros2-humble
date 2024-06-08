import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class FieldPublisher(Node):
    def __init__(self):
        super().__init__('field_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'soccer_field', 1)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # �p�����[�^��錾
        self.declare_parameter('field_length', 22.0)
        self.declare_parameter('field_width', 14.0)
        self.declare_parameter('goal_area_length', 0.75)
        self.declare_parameter('goal_area_width', 3.9)
        self.declare_parameter('penalty_area_length', 2.25)
        self.declare_parameter('penalty_area_width', 6.9)
        self.declare_parameter('line_width', 0.125)
        
        # ROS�p�����[�^����t�B�[���h�̃T�C�Y���擾
        self.field_length = self.get_parameter('field_length').get_parameter_value().double_value
        self.field_width = self.get_parameter('field_width').get_parameter_value().double_value
        self.goal_area_length = self.get_parameter('goal_area_length').get_parameter_value().double_value
        self.goal_area_width = self.get_parameter('goal_area_width').get_parameter_value().double_value
        self.penalty_area_length = self.get_parameter('penalty_area_length').get_parameter_value().double_value
        self.penalty_area_width = self.get_parameter('penalty_area_width').get_parameter_value().double_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value
    
    def timer_callback(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        # �t�B�[���h�̃T�C�Y��ROS�p�����[�^����擾�����l�ɒu��������
        field_length = self.field_length
        field_width = self.field_width
        
        # �S�[���G���A�̃T�C�Y��ROS�p�����[�^����擾�����l�ɒu��������
        goal_area_length = self.goal_area_length
        goal_area_width = self.goal_area_width
        
        # �y�i���e�B�G���A�̃T�C�Y��ROS�p�����[�^����擾�����l�ɒu��������
        penalty_area_length = self.penalty_area_length
        penalty_area_width = self.penalty_area_width

        # ���C���̕���ROS�p�����[�^����擾�����l�ɒu��������
        line_width = self.line_width
        
        # �^�b�`���C�� (�㉺)
        marker = self.create_line_marker(marker_id, [-field_length / 2.0, -field_width / 2.0 + line_width / 2.0], [field_length / 2.0, -field_width / 2.0 + line_width / 2.0], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [-field_length / 2.0, field_width / 2.0 - line_width / 2.0], [field_length / 2.0, field_width / 2.0 - line_width / 2.0], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # �S�[�����C�� (���E)
        marker = self.create_line_marker(marker_id, [-field_length / 2.0 + line_width / 2.0, -field_width / 2.0], [-field_length / 2.0 + line_width / 2.0, field_width / 2.0], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [field_length / 2.0 - line_width / 2.0, -field_width / 2.0], [field_length / 2.0 - line_width / 2.0, field_width / 2.0], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # �Z���^�[���C��
        marker = self.create_line_marker(marker_id, [0.0, -field_width / 2.0], [0.0, field_width / 2.0], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # �Z���^�[�T�[�N��
        center_diameter = 4.0 - line_width  # meters (���a 4m - �����̕�)
        marker = self.create_circle_marker(marker_id, center_diameter, width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # �S�[���G���A (��)
        goal_area_start_x_left = field_length / 2.0 - line_width / 2.0
        goal_area_end_x_left = goal_area_start_x_left - goal_area_length
        goal_area_y = goal_area_width / 2.0
        marker = self.create_line_marker(marker_id, [goal_area_start_x_left, -goal_area_y], [goal_area_end_x_left, -goal_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_start_x_left, goal_area_y], [goal_area_end_x_left, goal_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_end_x_left, -goal_area_y], [goal_area_end_x_left, goal_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # �S�[���G���A (�E)
        goal_area_start_x_right = -field_length / 2.0 + line_width / 2.0
        goal_area_end_x_right = goal_area_start_x_right + goal_area_length
        marker = self.create_line_marker(marker_id, [goal_area_start_x_right, -goal_area_y], [goal_area_end_x_right, -goal_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_start_x_right, goal_area_y], [goal_area_end_x_right, goal_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_end_x_right, -goal_area_y], [goal_area_end_x_right, goal_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # �y�i���e�B�G���A (��)
        penalty_area_start_x_left = field_length / 2.0 - line_width / 2.0
        penalty_area_end_x_left = penalty_area_start_x_left - penalty_area_length
        penalty_area_y = penalty_area_width / 2.0
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_left, -penalty_area_y], [penalty_area_end_x_left, -penalty_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_left, penalty_area_y], [penalty_area_end_x_left, penalty_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_end_x_left, -penalty_area_y], [penalty_area_end_x_left, penalty_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # �y�i���e�B�G���A (�E)
        penalty_area_start_x_right = -field_length / 2.0 + line_width / 2.0
        penalty_area_end_x_right = penalty_area_start_x_right + penalty_area_length
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_right, -penalty_area_y], [penalty_area_end_x_right, -penalty_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_right, penalty_area_y], [penalty_area_end_x_right, penalty_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_end_x_right, -penalty_area_y], [penalty_area_end_x_right, penalty_area_y], width=line_width)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        self.publisher.publish(marker_array)
        
    def create_line_marker(self, marker_id, start, end, width):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "soccer_field"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width  # Line width
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        
        p_start = self.create_point(float(start[0]), float(start[1]))
        p_end = self.create_point(float(end[0]), float(end[1]))
        marker.points.extend([p_start, p_end])
        
        return marker
    
    def create_circle_marker(self, marker_id, diameter, width):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "soccer_field"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width  # Line width
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        radius = diameter / 2.0
        points = []
        num_points = 100
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append(self.create_point(float(x), float(y)))
        
        # Close the circle
        points.append(points[0])
        
        marker.points.extend(points)
        
        return marker
    
    def create_point(self, x, y):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        return point

def main(args=None):
    rclpy.init(args=args)
    node = FieldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

       

