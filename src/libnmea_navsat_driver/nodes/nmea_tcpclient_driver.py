import socket
import sys
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from libnmea_navsat_driver.driver import Ros2NMEADriver

class NMEATCPClientNode(Node):
    def __init__(self):
        super().__init__('nmea_tcp_client_node')
        self.publisher = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray,'/diagnostics', 10)

        self.driver = Ros2NMEADriver()

        try:
            self.gnss_ip = self.driver.declare_parameter("ip", "192.168.131.22").value
            self.gnss_port = self.driver.declare_parameter("port", 9001).value
            self.buffer_size = self.driver.declare_parameter("buffer_size", 4096).value
            self.hardware_id = self.driver.declare_parameter("hardware_id", "gnss_device").value
        except KeyError as e:
            self.get_logger().err("Parameter %s not found" % e)
            sys.exit(1)

        self.frame_id = self.driver.get_frame_id()

        self.get_logger().info(
            "Using gnss sensor with ip {} and port {}".format(self.gnss_ip, self.gnss_port)
        )

    def run(self):
        diag = DiagnosticArray() # PUT THIS AS A variable we update?
        diag.header.stamp = self.get_clock().now().to_msg()

        stat = DiagnosticStatus(name="TCP_Socket", level=DiagnosticStatus.OK, message="OK",hardware_id=self.hardware_id)

        while rclpy.ok():
            try:
                gnss_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                gnss_socket.connect((self.gnss_ip, self.gnss_port))
                stat.message = "Socket Connection to: (%s)"%self.gnss_ip

            except socket.error as exc:
                # Publish diagnostics before killing node
                stat.message = "Socket Connection to (%s) Failed"%self.gnss_ip
                stat.level = DiagnosticStatus.ERROR
                diag.status.append(stat)
                self.diag_pub.publish(diag)
                self.get_logger().error(
                    "Caught exception socket.error when setting up socket: %s" % exc
                )
                sys.exit(1)
            diag.status.append(stat)
            self.diag_pub.publish(diag)


            gnss_stat = DiagnosticStatus(name="gnss_fix", level=DiagnosticStatus.OK, message="OK",hardware_id=self.hardware_id)

            partial = ""
            while rclpy.ok():
                try:
                    received_data = gnss_socket.recv(self.buffer_size).decode("ascii")
                    if len(received_data) == 0:
                        # Socket closed, connection lost
                        self.get_logger().error("Socket closed. Connection lost.")
                        stat.message = "Socket closed. Connection lost."
                        stat.level = DiagnosticStatus.ERROR
                        diag.status.clear()
                        gnss_down = DiagnosticStatus(name="gnss_fix", level=DiagnosticStatus.ERROR, message="Hardware signal lost",hardware_id=self.hardware_id)
                        diag.status.append(stat)
                        diag.status.append(gnss_down)
                        self.diag_pub.publish(diag)
                        gnss_socket.close()
                        break

                    partial += received_data
                    lines = partial.splitlines()
                    if partial.endswith("\n"):
                        full_lines = lines
                        partial = ""
                    else:
                        full_lines = lines[:-1]
                        partial = lines[-1]
                    diag.header.stamp = self.get_clock().now().to_msg()
                    for data in full_lines:
                        try:
                            if self.driver.add_sentence(data, self.frame_id):
                                gnss_stat.message = "GPS Fix"
                                gnss_stat.level = DiagnosticStatus.OK
                                pass
                            else:
                                self.get_logger().warn("Error with sentence: %s" % data)
                                gnss_stat.message = "No GPS Fix: Error with sentence: %s" % data
                                gnss_stat.level = DiagnosticStatus.WARN
                        except ValueError as e:
                            self.get_logger().warn(
                                "Value error, likely due to missing fields in the NMEA message. "
                                "Error was: %s. Please report this issue. " % e
                            )
                            gnss_stat.message = "No GPS Fix: Value error, likely due to missing fields in the NMEA message. Error was: %s. Please report this issue. " % e
                            gnss_stat.level = DiagnosticStatus.WARN
                        diag.status.clear()
                        diag.status.append(gnss_stat)
                        diag.status.append(stat)
                        self.diag_pub.publish(diag)

                except socket.error as exc:
                    self.get_logger().error(
                        "Caught exception socket.error when receiving: %s" % exc
                    )
                    stat.message = "Socket Connection to (%s) Failed"%self.gnss_ip
                    stat.level = DiagnosticStatus.ERROR
                    diag.status.append(stat)
                    self.diag_pub.publish(diag)
                    gnss_socket.close()
                    break

            gnss_socket.close()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    nmea_tcp_client = NMEATCPClientNode()
    nmea_tcp_client.run()

if __name__ == '__main__':
    main()
