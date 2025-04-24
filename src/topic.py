import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher


from unitree_api.msg import Request, Response


class Topic(Node):
    def __init__(self, name: str):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(Request, "/api/motion_switcher/request", 10)
        self.subscriber = self.create_subscription(
            Response,
            "/api/motion_switcher/response",
            self.subscribe_cb,
            10,
        )
            
        self.name = name

    def publish(self, msg):
        self.publisher.publish(msg)

    def subscribe_cb(self, msg):
        self.get_logger().info(f"Received message: {msg}")
        # Process the received message here
        # For example, you can call a callback function or update a variable


if __name__ == "__main__":

    rclpy.init()
    # Example usage
    topic = Topic("/api/motion_switcher/request")
    msg = Request()
    topic.publish(msg)
    rclpy.spin(topic)
    topic.destroy_node()
    rclpy.shutdown()
    