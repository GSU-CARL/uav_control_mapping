#!/usr/bin/env python3
"""Phase 1: Waypoint collection and visualization from RViz2 /clicked_point."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger



class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.waypoints = []  

        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_cb, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # Service to return collected waypoints
        self.srv = self.create_service(Trigger, '/generate_flight_path', self.generate_path_cb)
        
        # Service to clear waypoints
        self.clear_srv = self.create_service(Trigger, '/clear_waypoints', self.clear_waypoints_cb)

        self.get_logger().info('WaypointManager ready. Click points in RViz2.')

    def clear_waypoints_cb(self, request, response):
        self.waypoints.clear()
        
        # Publish DELETEALL marker
        markers = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        markers.markers.append(m)
        self.marker_pub.publish(markers)
        
        response.success = True
        response.message = "Waypoints cleared."
        return response

    def generate_path_cb(self, request, response):
        import json
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints collected yet."
        else:
            response.success = True
            response.message = json.dumps(self.waypoints)
            self.get_logger().info(f"Generated flight path with {len(self.waypoints)} waypoints.")
        return response

    def clicked_point_cb(self, msg: PointStamped):
        pt = (msg.point.x, msg.point.y, msg.point.z)
        self.waypoints.append(pt)
        self.get_logger().info(f'Waypoint {len(self.waypoints)}: {pt}')
        self.publish_markers()

    def publish_markers(self):
        markers = MarkerArray()

        # Sphere markers for each waypoint
        for i, (x, y, z) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.a = 1.0
            markers.markers.append(m)

        # LINE_STRIP connecting waypoints
        if len(self.waypoints) >= 2:
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = 'waypoint_path'
            line.id = 9999
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color.b = 1.0
            line.color.a = 1.0
            for (x, y, z) in self.waypoints:
                from geometry_msgs.msg import Point
                p = Point()
                p.x, p.y, p.z = x, y, z
                line.points.append(p)
            markers.markers.append(line)

        self.marker_pub.publish(markers)

    def get_waypoints(self):
        return self.waypoints


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()