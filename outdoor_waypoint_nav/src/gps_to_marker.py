#! /usr/bin/env python


import rospy 
from sensor_msgs.msg  import  NavSatFix
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from visualization_msgs.msg import Marker, MarkerArray

import tf

def publish_arrays(arrays, pub, frame='world', ns='none', time=None,
                  color=(0, 1, 0),marker_id=0,scale=0.1):
        
    #for index in range(len(corners)):
    markerArray = MarkerArray()
    for index in range(len(arrays)):
        mrk = Marker()
        mrk.id = marker_id
        mrk.header.frame_id = frame
        mrk.header.stamp = rospy.Time.now()
        mrk.type = mrk.POINTS
        mrk.action = mrk.ADD
        mrk.pose.position.x = 0.0
        mrk.pose.position.y = 0.0
        mrk.pose.position.z = 0.0
        mrk.pose.orientation.x = 0.0
        mrk.pose.orientation.y = 0.0
        mrk.pose.orientation.z = 0.0
        mrk.pose.orientation.w = 1.0
        mrk.scale.x = scale
        mrk.scale.y = scale
        mrk.scale.z = 0.0
        mrk.color.r = 0
        mrk.color.g = 1
        mrk.color.b = 0
        mrk.color.a = 1.0
        for i in range(len(arrays[index])):
            mrk.points.append(Point(arrays[index][1], arrays[index][0], 0))
        markerArray.markers.append(mrk)
        marker_id = marker_id+1
    pub.publish(markerArray)

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
        self.th = 0.0
        self.path = rospy.get_param("~coordinates_file")
        self.gps_array = []
        self.marker_pub     = rospy.Publisher('waypoints',MarkerArray,queue_size=1)
        sub = rospy.Subscriber('/navsat/fix',NavSatFix,self.callback,queue_size=100)
        with open(self.path) as f:
            for line in f:
                self.convert_to_XY(line.split(' '))
                # self.gps_array.append([float(elt) for elt in line.split(' ')])
        
        pass

    def callback(self, data):
        print("+++++++++++++++++++++++")
        publish_arrays(self.gps_array, self.marker_pub, 'map')

    def convert_to_XY(self, arr): 
        # deltaLatitude = p.latitude - relativeNullPoint.latitude
        # deltaLongitude = p.longitude - relativeNullPoint.longitude
        # latitudeCircumference = 40075160 * cos(asRadians(relativeNullPoint.latitude))
        self.resultX = float(arr[0]) * 40008000 / 360
        self.resultY = float(arr[1])  * 40008000 / 360
        resultX = float(arr[0]) * 40008000 / 360
        resultY = float(arr[1])  * 40008000 / 360

        # print(self.resultX,self.resultY)
        
        
        self.current_position(resultX, resultY)
       

    def current_position(self, X, Y):
        if self.init_pos_x !=None and self.init_pos_y != None:
            dt = (self.current_time - self.last_time).to_sec()
            print("time diff", dt)
            # self.vx = (self.resultX- self.init_pos_x - self.x )/dt
            # self.vy = (self.resultY- self.init_pos_y - self.y )/dt
            # if self.vx<0.05 and self.vx>-0.05: 
            #     self.vx=0.0
            # if self.vy<0.05 and self.vy>-0.05: 
            #     self.vy=0.0

            X = X - self.init_pos_x 
            Y = Y - self.init_pos_y
            self.gps_array.append([X, Y])
            

            # Now we can publish the position based on the robot movenment
            

        elif self.resultX != None and self.resultY != None: 
            self.init_pos_x = self.resultX
            self.init_pos_y = self.resultY 
        
       
        
 
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
    
    
        