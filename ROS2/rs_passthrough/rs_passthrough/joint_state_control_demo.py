"""ROS2/rs_passthrough/rs_passthrough/velocity_control_demo.py"""

import rclpy
import time
import struct

from rclpy.node import Node
from rs_msgs.msg import Packet
from sensor_msgs.msg import JointState

from rs_protocol import RSProtocol, PacketID, decode_floats


class JointStatesExample(Node):
    def __init__(self):
        super().__init__("velocity_control_example")

        self.declare_parameter("~frequency", 20)
        self.declare_parameter('device_id', 0x01)
        self.declare_parameter('joint_name', "alpha_axis_a")
        self.frequency = 20
        self.start_time = self._clock.now()
        self.tx_publisher = self.create_publisher(Packet, "tx", 100)
        self.rx_subscriber = self.create_subscription(Packet, "rx", self.receive_packet, 100)
        self.joint_states_subscriber = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 100)
        self.joint_name = self.get_parameter('joint_name').value
        self.device_id = self.get_parameter('device_id').value
        self.request_packet = Packet()
        self.request_packet.device_id = self.device_id
        self.request_packet.packet_id = int(PacketID.POSITION)
        self.request_packet.data = list(struct.pack('f', 0.0))

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

    def joint_state_callback(self, joint_states_msg):
        if self.joint_name in joint_states_msg.name:
            index = joint_states_msg.name.index(self.joint_name)
            position = joint_states_msg.position[index]
            self.request_packet.data = self.interpolate_input(position)

    def interpolate_input(self, value, in_min = 0.0, in_max = 0.015, out_min = 0, out_max = 13):
        if self.device_id == 0x01:
            in_min = 0.0
            in_max = 0.015
            out_min = 0
            out_max = 13
        elif self.device_id == 0x02:
            in_min = 0.0
            in_max = 6.2
            out_min = 0
            out_max = 3.14
        elif self.device_id == 0x03:
            in_min = 0.0
            in_max = 3.5
            out_min = 0
            out_max = 3.14
        elif self.device_id == 0x04:
            in_min = 0.0
            in_max = 3.8
            out_min = 1.57
            out_max = 3.14
        

        value = max(in_min, min(value, in_max))
        return list(struct.pack('f',(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min))
    
    def timer_callback(self):
        self.get_logger().info("Device ID: {}, Packet ID: {}, Data: {}".format(self.request_packet.device_id, self.request_packet.packet_id, self.request_packet.data))
        self.tx_publisher.publish(self.request_packet)

    def receive_packet(self, packet):   
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)

        if packet_id == PacketID.VELOCITY:
                velocity = decode_floats(data)[0]
                self.get_logger().info("Velocity Received: {} - {}".format(device_id, velocity))

def main(args=None):
        rclpy.init(args=args)
        vce = JointStatesExample()
        rclpy.spin(vce)

if __name__ == "__main__":
         main()