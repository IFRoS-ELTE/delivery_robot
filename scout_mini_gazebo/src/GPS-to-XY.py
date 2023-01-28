#! /usr/bin/env python


import rospy 
from sensor_msgs.msg  import  NavSatFix
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import tf

class scout: 
    def __init__(self):
        print("Node initialized correctly ")
        self.init_pos_x = None
        self.init_pos_y = None 

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.x= 0 
        self.y=0
        #Note that the Thita will not update because the GPS cannot provide orientation
        self.th = 0 

        self.vx=0.0
        self.vy=0.0
        self.vth = 0.0

        self.GPS_odom_pub = rospy.Publisher('GPS_odom', Odometry, queue_size=10)
        odom_broadcaster = tf.TransformBroadcaster()
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        sub = rospy.Subscriber('/gps/fix',NavSatFix,self.callback,queue_size=100)

        
        pass

    def callback(self, data):
       
        self.lat_pos = data.latitude
        self.long_pos = data.longitude
        self.covariance = data.position_covariance

        self.convert_to_XY()

    def convert_to_XY(self): 
        # deltaLatitude = p.latitude - relativeNullPoint.latitude
        # deltaLongitude = p.longitude - relativeNullPoint.longitude
        # latitudeCircumference = 40075160 * cos(asRadians(relativeNullPoint.latitude))
        self.resultX = self.long_pos * 40008000 / 360
        self.resultY = self.lat_pos  * 40008000 / 360

        # print(self.resultX,self.resultY)
        
        
        self.current_position()
       

    def current_position(self):
        
        self.current_time = rospy.Time.now()
        if self.init_pos_x !=None and self.init_pos_y != None:
            dt = (self.current_time - self.last_time).to_sec()
            print("time diff", dt)
            self.vx = (self.resultX- self.init_pos_x - self.x )/dt
            self.vy = (self.resultY- self.init_pos_y - self.y )/dt
            if self.vx<0.05 and self.vx>-0.05: 
                self.vx=0.0
            if self.vy<0.05 and self.vy>-0.05: 
                self.vy=0.0

            self.x = self.resultX- self.init_pos_x 
            self.y = self.resultY- self.init_pos_y
            

            # Now we can publish the position based on the robot movenment
            self.publisher_odom()

        elif self.resultX != None and self.resultY != None: 
            self.init_pos_x = self.resultX
            self.init_pos_y = self.resultY 
      

        
    # publisher_odom(current_position_x,current_position_y)

    def publisher_odom(self):  
        print(" the computed position is", self.x , "and ", self.y )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position 
        odom.pose.pose = Pose(Point(-self.x, self.y, 0.), Quaternion(*self.odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(-self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.GPS_odom_pub.publish(odom)
        

        self.last_time= rospy.Time.now()

        
 
def main():
    '''Initializes and cleanup ros node'''
    rospy.init_node('GPS_XY', anonymous=True)
    robot = scout()

    
    try:
        rospy.spin()
    except KeyboardInterrupt:
       print ("Shutting down ROS GPS to XY odom publisher")

   

if __name__=='__main__': 
    
    main()
    
    
        