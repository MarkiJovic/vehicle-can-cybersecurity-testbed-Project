#!/usr/bin/env python3

import rospy
import can
import threading
import time
from std_msgs.msg import Header
from can_msgs.msg import Frame

class FixedCANBridge:
    def __init__(self):
        rospy.init_node('fixed_can_bridge_v3', anonymous=True)
        
        # CAN bus setup
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)
            rospy.loginfo("CAN bridge initialized successfully")
        except Exception as e:
            rospy.logerr(f"CAN setup failed: {e}")
            return
            
        # Publishers
        self.can_rx_pub = rospy.Publisher('/can_rx', Frame, queue_size=10)
        
        # Start CAN reading thread
        self.running = True
        self.can_thread = threading.Thread(target=self.read_can_loop)
        self.can_thread.daemon = True
        self.can_thread.start()
        
    def read_can_loop(self):
        """Read CAN messages in separate thread"""
        while self.running:
            try:
                message = self.bus.recv(timeout=0.001)
                if message:
                    # Convert CAN message to ROS Frame
                    ros_frame = Frame()
                    ros_frame.id = message.arbitration_id
                    ros_frame.dlc = message.dlc
                    
                    # Pad data to 8 bytes if needed
                    data_list = list(message.data)
                    while len(data_list) < 8:
                        data_list.append(0)
                    ros_frame.data = data_list[:8]  # Ensure exactly 8 bytes
                    
                    # Publish to /can_rx for engine node
                    self.can_rx_pub.publish(ros_frame)
                    
                    rospy.loginfo(f"CAN RX: ID={hex(message.arbitration_id)}, DLC={message.dlc}, data={data_list[:8]}")
                    
            except can.CanError:
                pass
            except Exception as e:
                rospy.logwarn(f"CAN read error: {e}")
                
    def cleanup(self):
        self.running = False
        if hasattr(self, 'bus'):
            self.bus.shutdown()

if __name__ == '__main__':
    try:
        bridge = FixedCANBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        bridge.cleanup()
