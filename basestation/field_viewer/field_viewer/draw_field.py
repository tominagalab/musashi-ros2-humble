import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

class FieldPublisher(Node):
    def __init__(self):
        super().__init__('field_publisher')
        self.publisher = self.create_publisher(MarkerArray, 'soccer_field', 1)  # パブリッシュレートを1Hzに変更
        timer_period = 1.0  # seconds  # タイマーを1秒に変更
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        marker_array = MarkerArray()
        marker_id = 0
        
        # フィールドのサイズを定義（線の幅を考慮して調整）
        field_length = 22.0  # meters (タッチライン)
        field_width = 14.0  # meters (ゴールライン)
        
        # タッチライン (上下)
        marker = self.create_line_marker(marker_id, [-field_length / 2.0, -field_width / 2.0 + 0.0625], [field_length / 2.0, -field_width / 2.0 + 0.0625], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [-field_length / 2.0, field_width / 2.0 - 0.0625], [field_length / 2.0, field_width / 2.0 - 0.0625], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # ゴールライン (左右)
        marker = self.create_line_marker(marker_id, [-field_length / 2.0 + 0.0625, -field_width / 2.0], [-field_length / 2.0 + 0.0625, field_width / 2.0], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [field_length / 2.0 - 0.0625, -field_width / 2.0], [field_length / 2.0 - 0.0625, field_width / 2.0], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # センターライン
        marker = self.create_line_marker(marker_id, [0.0, -field_width / 2.0], [0.0, field_width / 2.0], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # センターサークル
        center_diameter = 4.0 - 0.125*2  # meters (直径 4m - 線幅の分)
        marker = self.create_circle_marker(marker_id, center_diameter, width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # ゴールエリア (左)
        goal_area_width = 3.9  # meters
        goal_area_length = 0.75  # meters
        goal_area_start_x_left = field_length / 2.0 - 0.0625
        goal_area_end_x_left = goal_area_start_x_left - goal_area_length
        goal_area_y = goal_area_width / 2.0
        marker = self.create_line_marker(marker_id, [goal_area_start_x_left, -goal_area_y], [goal_area_end_x_left, -goal_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_start_x_left, goal_area_y], [goal_area_end_x_left, goal_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_end_x_left, -goal_area_y], [goal_area_end_x_left, goal_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # ゴールエリア (右)
        goal_area_start_x_right = -field_length / 2.0 + 0.0625
        goal_area_end_x_right = goal_area_start_x_right + goal_area_length
        marker = self.create_line_marker(marker_id, [goal_area_start_x_right, -goal_area_y], [goal_area_end_x_right, -goal_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_start_x_right, goal_area_y], [goal_area_end_x_right, goal_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [goal_area_end_x_right, -goal_area_y], [goal_area_end_x_right, goal_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # ペナルティエリア (左)
        penalty_area_length = 2.25  # meters
        penalty_area_width = 6.9  # meters
        penalty_area_start_x_left = field_length / 2.0 - 0.0625
        penalty_area_end_x_left = penalty_area_start_x_left - penalty_area_length
        penalty_area_y = penalty_area_width / 2.0
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_left, -penalty_area_y], [penalty_area_end_x_left, -penalty_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_left, penalty_area_y], [penalty_area_end_x_left, penalty_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_end_x_left, -penalty_area_y], [penalty_area_end_x_left, penalty_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        
        # ペナルティエリア (右)
        penalty_area_start_x_right = -field_length / 2.0 + 0.0625
        penalty_area_end_x_right = penalty_area_start_x_right + penalty_area_length
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_right, -penalty_area_y], [penalty_area_end_x_right, -penalty_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_start_x_right, penalty_area_y], [penalty_area_end_x_right, penalty_area_y], width=0.125)
        marker.color.r = 1.0  # Red color
        marker.color.g = 1.0  # Green color
        marker.color.b = 1.0  # Blue color
        marker_array.markers.append(marker)
        marker_id += 1
        marker = self.create_line_marker(marker_id, [penalty_area_end_x_right, -penalty_area_y], [penalty_area_end_x_right, penalty_area_y], width=0.125)
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

       

