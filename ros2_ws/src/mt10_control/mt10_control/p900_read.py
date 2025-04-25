import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from rtcm_msgs.msg import Message
from sbg_driver.msg import SbgGpsPos
import serial
from rclpy.timer import Timer

class SerialCommunicator(Node):
    def __init__(self):
        super().__init__('serial_communicator')
        
        # Setup serial communication
        self.ser = serial.Serial('/dev/ttyUSB0', 56700, timeout=1)  # Adjust port and baudrate as needed

        # Timer to read from serial every 1 second
        self.timer = self.create_timer(0.0001, self.read_from_serial)
        
        # Subscriber for sending host machine topic data
        self.rtk_sub = self.create_subscription(Message, "/rtcm", self.rtk_sub_callback, 1)
       
       # Create Publisher for converting serial data to ros data
        self.witmotion_pub = self.create_publisher(Float64, "/witmotion_eular/yaw", 1)
        self.gps_pos_pub = self.create_publisher(SbgGpsPos, "/sbg/gps_pos", 1)
        self.status_pub = self.create_publisher(String, "/status", 1)
        

    def read_from_serial(self):
        """ Read data from the serial device and publish it if available """
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()  # Read and decode serial data
            print(data)
            self.get_logger().info(f'Read from serial: {data}')

            topic, ros_data = data.split("#")

            if topic == "/witmotion_eular":
                msg = Float64()
                msg.data = int(ros_data)
                self.witmotion_pub.publish(msg)

            elif topic == "/sbg/gps_pos":
                msg = SbgGpsPos()
                latitude,longitude,positional_accuracy_x,positional_accuracy_y,positional_accuracy_z = ros_data.split("*")
                msg.latitude = float(latitude)
                msg.longitude = float(longitude)
                msg.position_accuracy.x = float(position_accuracy_x)
                msg.position_accuracy.y = float(position_accuracy_y)
                msg.position_accuracy.z = float(position_accuracy_z)
                self.gps_pos_pub.publish(msg)

            elif topic == "/status":
                msg = String()
                msg.data = ros_data
                self.status_pub.publish(msg)

    def write_to_serial(self, msg: String):
       """ Write received message to the serial device """
       self.get_logger().info(f'Writing to serial: {msg}')
       msg.data += "\n"
       self.ser.write(msg.data.encode('utf-8'))  # Write data to serial
        
    def rtk_sub_callback(self, msg: Message):
        """ Callback for the RTCM message subscriber """
        # Convert the RTCM message to a string and write it to the serial device
        str_msg = String()
        str_msg.data = "/rtcm#"
        str_msg.data += str(msg.data)
        self.write_to_serial(str_msg.data)
    	
    
    # def gps_callback(self, msg: SbgGpsPos):
    #   lat = str(msg.latitude)
    #   lon = str(msg.longitude)
    #   pos_acc_x = str(msg.position_accuracy.x)
    #   pos_acc_y = str(msg.position_accuracy.y)
    #   pos_acc_z = str(msg.position_accuracy.z)

    #   str_msg = String()

    #   str_msg.data = "/sbg/gps_pos#"

    #   str_msg.data += f"{lat}*{lon}*{pos_acc_x}*{pos_acc_y}*{pos_acc_z}"

    #   self.write_to_serial(str_msg.data)
    	
    #def status_callback(self, msg: String):
    #    str_msg = String()
    #    str_msg.data = "/status#"

    #   str_msg.data += msg.data

    #    self.write_to_serial(str_msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunicator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()  # Close the serial port when shutting down
        rclpy.shutdown()

if __name__ == '__main__':
    main()

