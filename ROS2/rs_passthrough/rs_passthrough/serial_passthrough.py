"""
serial_passthrough.py

Used to connect to an arm, and forward and ros messages received and
"""
import serial
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile

from rs_msgs.msg import Packet
from rs_protocol import RSProtocol, create_serial_connection


class RSPassthroughNode(Node):

    def __init__(self):
        super().__init__('serial_passthrough')
        self.rx_publisher = self.create_publisher(Packet,'rx', 5)

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        serial_port = self.get_parameter('serial_port').value
        self.tx_subscriber = self.create_subscription(Packet, "tx", self.tx_transmit, QoSProfile(depth=5))

        if not serial_port:
            raise ValueError("No serial port speicifed. Please declare the serial port via ros command line arguments")

        try:
            self.rs_protocol = RSProtocol(create_serial_connection(serial_port))
            self.get_logger().info("Opened serial port {}".format(serial_port))
        except serial.SerialException as e:
            self.get_logger().error("Unable to open serial port {}".format(e))
            self.destroy_node()
            return
        
        self.timer = self.create_timer(1/1000, self.rx_receive)
        # self.rx_receive()

    def tx_transmit(self, packet):
        self.get_logger().debug("Transmitting {}, {}, {}".format(packet.device_id, 
                                                                 packet.packet_id, 
                                                                 packet.data))
        self.rs_protocol.write(packet.device_id, packet.packet_id, packet.data)


    def rx_receive(self):
        
        # rate = rospy.Rate(10000)
        # while rclpy.ok():
        #     rclpy.spin_once(self)
            # time.sleep(0.000000001)
    
        # 1. Read using rs_protocol 
        # 2. List of packets returned from rs_protocol
        # 3. For each packet, create a ros packet and publish it
        try:
            packets = self.rs_protocol.read()
        except serial.SerialException as e:
            self.get_logger().error("Error reading from serial: {}".format(e))
            return
        
        if packets:
            for packet in packets:
                device_id = packet[0]
                packet_id = packet[1]
                data = packet[2]

                ros_packet = Packet()
                ros_packet.device_id = device_id
                ros_packet.packet_id = packet_id
                ros_packet.data = list(data)
                self.rx_publisher.publish(ros_packet)
        # Read from serial and transmit
        pass


def main(args=None):
    rclpy.init(args=args)
    passthrough_node = RSPassthroughNode()
    rclpy.spin(passthrough_node)


if __name__ == "__main__":
    main()