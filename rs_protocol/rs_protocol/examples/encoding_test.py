import time
from rs_protocol import RSProtocol, PacketID, Mode, encode_single_packet, parse_packet
import logging, struct
from typing import List
logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

# Device configuration
DEVICE_ID = 0x01
PACKET_ID = 0x03  # Example packet ID 
DATA = [1.0]  # Example data to encode


def encoding_data():
    # Example of encoding a packet with a single float value
    encodedPacket = encode_single_packet(DEVICE_ID, PACKET_ID, DATA)
    print("banana")
    return encodedPacket

def encode_floats(float_list: float) -> bytes:
    """ Decode a received byte list, into a float list as specified by the rr protocol"""
    # data = struct.pack('%sf' % len(float_list), *float_list)
    data = struct.pack('<f', float_list)
    print(f"Encoded data: {data}")
    return data


def decoding_data():
    # Example of decoding a packet
    rawData = encoding_data()
    print(f"Raw Packet Data: {rawData}")
    decoded_data = parse_packet(rawData)
    print(f"Parsed Packet: {decoded_data}")
    

def main():
    print("\n--- Encoding and Decoding Test ---")
    print(f"Device ID: {DEVICE_ID}"
            f"\nPacket ID: {PACKET_ID}"
            f"\nData: {DATA}"
          )
    # print(f"encode: {encode_floats(DATA)}") 
    decoding_data()
    encode_floats(1.0)

if __name__ == '__main__':
    main()