
"""
udp_passthrough.py

Used to connect to an arm, and forward and ros messages received and
"""

import rclpy
import socket
import struct 

from rclpy.node import Node
from rclpy.qos import QoSProfile

from rs_msgs.msg import Packet
from rs_protocol import RSProtocol, PacketID, create_socket_connection


class RSPassthroughNode(Node):

    def __init__(self):
        super().__init__('udp_passthrough')

        self.rx_publisher = self.create_publisher(Packet,'rx', 5)

        self.tx_subscriber = self.create_subscription(Packet, 'tx', self.tx_transmit, qos_profile=QoSProfile(depth=5))

        self.declare_parameter('ip_address', '192.168.2.3')
        self.declare_parameter('port', 6789)

        self.ip_address = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        try:
            self.rs_protocol = RSProtocol(create_socket_connection(), (self.ip_address, self.port))
        except socket.error as e:
            self.get_logger().error("Unable to open socket connection: {}".format(e))
            self.destroy_node()
            return
        #self.sock.setblocking(False)
        #self.sock.bind(("", 0))

        self.get_logger().info("Opened Socket to ip {} at port {}".format(self.ip_address, self.port))
      
        self.timer = self.create_timer(1/1000000, self.rx_receive)
        
        self.rx_receive()
        pass

    def tx_transmit(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        if packet_id == PacketID.VELOCITY | packet_id == PacketID.POSITION:
            data = struct.unpack('f', packet.data)[0]
        else:
            data = list(packet.data)
        self.get_logger().info("Transmitting {}, {}, {}".format(device_id, 
                                                                 packet_id, 
                                                                 data))
        self.get_logger().info("Data: {}".format(data))
        self.rs_protocol.write(device_id, packet_id, data)
        

    def rx_receive(self):
        # packet_reader = PacketReader()
        # rate = rospy.Rate(10000)
        try:
            
            raw_packets = self.rs_protocol.read_raw()
        except socket.error as e:
            self.get_logger().error("Error reading from socket: {}".format(e))
            return
        
        if raw_packets:
            self.get_logger().info("Packets: {}".format(raw_packets))
            for packet in raw_packets:
                device_id = packet[0]
                packet_id = packet[1]
                data = packet[2]

                ros_packet = Packet()
                ros_packet.device_id = device_id
                ros_packet.packet_id = packet_id
                ros_packet.data = list(data)
                # print(data)
                # self.get_logger().info("Publishing {}".format(ros_packet))
                self.rx_publisher.publish(ros_packet)

        # rate.sleep()
        pass

def main(args=None):
    rclpy.init(args=args)
    passthrough_node = RSPassthroughNode()
    rclpy.spin(passthrough_node)

if __name__ == '__main__':
    main()
