#! /usr/bin/env python
import rospy 
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

br = tf.TransformBroadcaster()


rospy.init_node('odom_pub')

odom_pub =  rospy.Publisher('odom', Odometry)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = 'odom'

model = GetModelStateRequest()
model.model_name = 'scout/'

r=rospy.Rate(2)

while not rospy.is_shutdown():

    result = get_model_srv(model)
    
    # print(type(result))
    
    odom.pose.pose = result.pose
    odom.twist.twist = result.twist
    
    header.stamp = rospy.Time.now() 
    odom.header = header 
    
    # print(odom.pose.pose.position.x)
    
    
    odom_pub.publish(odom)
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, 0.23),
                      (odom.pose.pose.orientation.x, 
                       odom.pose.pose.orientation.y, 
                       odom.pose.pose.orientation.z, 
                       odom.pose.pose.orientation.w),
                      rospy.Time.now(),
                      "base_link",
                      "odom")
    
    r.sleep()
    
