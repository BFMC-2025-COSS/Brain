import rospy
import serial
from std_msgs.msg import Float32

class SerialReadNode:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        rospy.init_node("serial_read_node", anonymous=True)
        rospy.loginfo("Serial Read Node Started")

        self.serial_port = serial.Serial(port, baudrate, timeout=0.1)

        self.speed_pub = rospy.Publisher("/sensor/speed",Float32,queue_size=10)

    def sendqueue(self,buff):
        try:
            action, value = buff.split(":") # @action:value;;
            action = action[1:]
            value = value[:-2]

            if action == "speedSensor":
                speed = value.split(",")[0]
                speed = float(speed)

                #rospy.loginfo(f"Received speed: {speed}")
                self.speed_pub.publish(speed)
        except Exception as e:
            rospy.logerr(f"Error processing serial data: {e}")

    def run(self):
        buff =""
        isResponse = False

        while not rospy.is_shutdown():
            read_chr = self.serial_port.read()
            try:
                read_chr = read_chr.decode("ascii")
                if read_chr == "@":
                    isResponse = True
                    buff = ""
                elif read_chr == "\r":
                    isResponse = False
                    if len(buff) != 0:
                        self.sendqueue(buff)
                if isResponse:
                    buff += read_chr
            except Exception as e:
                print(e)
            
if __name__ == "__main__":
    node = SerialReadNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass