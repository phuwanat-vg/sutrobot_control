#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float32,Int32,Int16
from geometry_msgs.msg import Twist,Pose2D,TransformStamped
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3
import math
import tf2_ros
import tf


class Robot():

    def __init__(self):
       
        self.wheel_radius = 0.04225 #m
        self.wheelbase = 0.177 #m
        self.PPR = 840 #m

        self.leftwheel_power_pub = rospy.Publisher('sutrobot1/wheel_power_left',Float32, queue_size = 10)
        self.rightwheel_power_pub = rospy.Publisher('sutrobot1/wheel_power_right',Float32, queue_size =10)
       
        self.vel_sub = rospy.Subscriber('sutrobot1/cmd_vel', Twist, self.vel_callback,queue_size = 10)
        self.v = 0
        self.w = 0
  
        self.rpm_max = 29.0 #RPM
        
        
        self._wheel_vel = {"r":0.0,"l":0.0}
        self.power_left = Float32()
        self.power_right = Float32()
        self.power_left.data = 0.0
        self.power_right.data = 0.0
        
        rospy.loginfo("start robot core node")
        
        #Odometry parameters
        self.left_sub = rospy.Subscriber('sutrobot1/tick_wheel_left', Int32, self.left_tick_cb)
        self.right_sub = rospy.Subscriber('sutrobot1/tick_wheel_right', Int32, self.right_tick_cb)
        self.rpm_left_sub = rospy.Subscriber('sutrobot1/rpm_left',Float32, self.rpm_left_cb)
        self.rpm_right_sub = rospy.Subscriber('sutrobot1/rpm_right',Float32, self.rpm_right_cb)
        self._cur_tick = {"r":None, "l":None}
        self._cur_tick_ts = {"r":None, "l":None}
        self.prev_tick = {"r":None,"l":None}
        
        #Odometry
        self.last_time = rospy.Time.now()
        self.dx = Float32()
        self.dy = Float32()
        self.dtheta = Float32()
        self.odom_pub = rospy.Publisher('odom',Odometry, queue_size=20)
        self.pose2D = Pose2D()
        self.actual_left_vel = 0.0
        self.actual_right_vel = 0.0
        #TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
      
        self.rate = rospy.Rate(20)
        rospy.loginfo("Ready for control")
    
    def rpm_left_cb(self,rpm):
        self.actual_left_vel = rpm.data*0.10472*self.wheel_radius
    def rpm_right_cb(self,rpm):
        self.actual_right_vel = rpm.data*0.10472*self.wheel_radius
        

    def vel_callback(self,vel):
   
        self.v = vel.linear.x
        self.w = vel.angular.z
        L = self.wheelbase
        R = self.wheel_radius
        
        #convert unicycle to diff drive
        v_l = ((2.0*self.v)+(self.w*L))/(2.0*R)
        v_r = ((2.0*self.v)-(self.w*L))/(2.0*R)
        
        self.wheel_speed_setpoint(v_r,v_l)
        
    def convert_vel_RPM(self,v):
        v_mag = v
        
        power = v/(2*3.14159)*60
        
        if power > self.rpm_max:
        	power = 29.0
        else:
        	power = power
        
        return power
    
    def wheel_speed_setpoint(self,vr,vl):
         #vr = max(min(vr,self.max_speed),self.max_speed*-1)
         #vl = max(min(vl,self.max_speed),self.max_speed*-1)
         #if vr == 0.0 and vl == 0.0:
            #for index in ["r","l"]:
            #    self._wheel_vel[index] = 0.0
            #self.power_left.data = 0.0
            #self.power_right.data = 0.0
            
         self.power_left.data = self.convert_vel_RPM(vr)
         self.power_right.data = self.convert_vel_RPM(vl)
         
         self.leftwheel_power_pub.publish(self.power_left.data)
         self.rightwheel_power_pub.publish(self.power_right.data) 
         
    def left_tick_cb(self, tick):
        self.odometry()
        self._cur_tick["l"]=tick.data
        self._cur_tick_ts["l"]=rospy.Time.now()
        #self.get_logger().info('time is: "%s"' %self._cur_tick_ts["l"])
        
    def right_tick_cb(self, tick):
        self._cur_tick["r"]=tick.data
        self._cur_tick_ts["r"]=rospy.Time.now()
        #self.get_logger().info('time is: "%s"' %self._cur_tick_ts["l"])
        
    def get_tick(self):
       
        tick = {}
        tick["l"] = self._cur_tick["l"]
        tick["r"] = self._cur_tick["r"]
        tick["rts"] = self._cur_tick_ts["r"]
        tick["lts"] = self._cur_tick_ts["l"]
        
        return tick
        
    def get_pose(self):
        return self.pose2D
        
    def update_pose(self, pose):
        self.pose2D.x = pose.x
        self.pose2D.y = pose.y
        self.pose2D.theta = pose.theta
    
    def odometry(self):
        ts = rospy.Time.now()
        tick = self.get_tick()
        dt = ts-self.last_time
        dt = dt.to_sec()
        #self.get_logger().info('dt is: "%s"' %dt)
        
        if tick["r"]==None or tick["l"]==None:
            return
            
        if self.prev_tick["r"]==None or self.prev_tick["l"]==None:
            self.prev_tick = {"r":tick["r"],"l":tick["l"]}
        
        prev_robot_pose = self.get_pose()
        R = self.wheel_radius
        L = self.wheelbase
        TPR = self.PPR
        DPT = (2*math.pi*R)/TPR #Distance per revolution
        
        DL = DPT*(tick["l"]-self.prev_tick["l"])
        
        DR = DPT*(tick["r"]-self.prev_tick["r"])
        
        DC = (DL+DR)*0.5
        
        self.dx = DC*math.cos(prev_robot_pose.theta)
        self.dy = DC*math.sin(prev_robot_pose.theta)
        self.dtheta = (DR-DL)/L
        new_robot_pose = Pose2D()
        new_robot_pose.x = prev_robot_pose.x+self.dx
        new_robot_pose.y = prev_robot_pose.y+self.dy
        theta_tmp  = prev_robot_pose.theta+self.dtheta
        new_robot_pose.theta = math.atan2(math.sin(theta_tmp), math.cos(theta_tmp))
        
        self.update_pose(new_robot_pose)
        
        self.prev_tick["r"]=tick["r"]
        self.prev_tick["l"]=tick["l"]
        
        v_ave = (self.actual_left_vel+self.actual_right_vel)/2.0
        v_dif = (self.actual_right_vel-self.actual_left_vel)/2.0

        vx = (v_ave/60.0)*2*3.14159*self.wheel_radius

        vy = 0.0

        w = (v_dif/60.0*(2*3.14159*self.wheel_radius))/(self.wheelbase/2.0)
            
        #prepare data for odom and tf
        pose = self.get_pose()
        quat = tf.transformations.quaternion_from_euler(0,0,pose.theta)
        
        #broadcast transform
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id= "base_link"
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)
        
        #publish odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        #Add pose to odom
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3] 
        #Add velocity to odom
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.z = w
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.x = 0.0

        odom.pose.covariance[0] = 0.0001;
        odom.pose.covariance[7] = 0.0001;
        odom.pose.covariance[35] = 0.01;
        odom.twist.covariance[0] = 0.00001;
        odom.twist.covariance[7] = 0.00001;
        odom.twist.covariance[35] = 0.01;

        self.odom_pub.publish(odom)

        #update last time
        self.last_time = ts
        
        #Debug Zone
        #print('rtick: "%s"' %tick)
        #rospy.loginfo('DR: "%s"' %DR)
        #rospy.loginfo('DL: "%s"' %DL)
        #rospy.loginfo('DC: "%s"' %DC)
        #rospy.loginfo('prev pose: "%s"' %prev_robot_pose)
        #rospy.loginfo('dx: "%s"' %self.dx)
        #rospy.loginfo('dy: "%s"' %self.dy)
        #rospy.loginfo('dth: "%s"' %self.dtheta)

def main(args=None):
    rospy.init_node('Robot_core', anonymous=True)
    robot = Robot()
    robot.rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
