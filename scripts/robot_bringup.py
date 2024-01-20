#! /usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32,Int32,Int16
from geometry_msgs.msg import Twist,Pose2D,TransformStamped
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Point,Quaternion,Vector3
import math
import tf2_ros
import tf
import numpy as np
import time
from sensor_msgs.msg import Imu

class Robot():

    def __init__(self):
       
        self.wheel_radius = 0.0696 #m
        self.wheelbase = 0.265 #m
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.vel_callback,queue_size = 10)
        self.v = 0
        self.w = 0

        self.arduino = serial.Serial(port="/dev/ttyUSB1", baudrate=115200, timeout=0.01)
        self.timer = rospy.Timer(rospy.Duration(1.0/200.0), self.mcu_callback)
        self.rpm_max = 55.0 #RPM


        self._wheel_vel = {"r":0.0,"l":0.0}
        self.power_left = Float32()
        self.power_right = Float32()
        self.power_left.data = 0.0
        self.power_right.data = 0.0

        rospy.loginfo("start robot core node")

        #Odometry
        self.last_time = rospy.Time.now()
        self.dx = 0.0
        self.dy = 0.0
        self.dtheta = 0.0
        self.odom_pub = rospy.Publisher('odom',Odometry, queue_size=20)
        self.imu_pub = rospy.Publisher('imu/data',Imu, queue_size=5)
        #IMU
        self.accel = [0.0,0.0,0.0]
        self.gyro = [0.0,0.0,0.0]
        self.imu_quat = [1.0, 0.0, 0.0, 0.0]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.robot_feedback_vel = [0.0,0.0]
        #TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        

        self.rate = rospy.Rate(20)
        rospy.loginfo("Ready for control")

    def mcu_callback(self, event=None):
        if self.arduino.is_open:
            
            
            try:
                raw_data = self.arduino.readline().decode('utf-8').strip()
            except:
                return
            

            values = raw_data.split(',')

            if values[0] == "ENC":
                #print("{} {} ".format(values[1], values[2]))
                pass
            elif values[0] == "RVL":
                print("{} {}".format( values[1], values[2] ))
                self.robot_feedback_vel = [values[1],values[2]]
                #pass
            elif values[0] == "POS":
                
                x = float(values[1])
                y = float(values[2])
                theta = float(values[3])
                quat = tf.transformations.quaternion_from_euler(0,0,theta)
        
                #broadcast transform
                t = TransformStamped()
                t.header.frame_id = "odom"
                t.child_frame_id= "base_link"
                t.header.stamp = rospy.Time.now()
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
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
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation.x = quat[0]
                odom.pose.pose.orientation.y = quat[1]
                odom.pose.pose.orientation.z = quat[2]
                odom.pose.pose.orientation.w = quat[3] 
                #Add velocity to odom
                odom.twist.twist.linear.x = self.robot_feedback_vel[0]
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.linear.z = 0.0

                odom.twist.twist.angular.z = self.robot_feedback_vel[1]
                odom.twist.twist.angular.y = 0.0
                odom.twist.twist.angular.x = 0.0

                odom.pose.covariance[0] = 0.0001
                odom.pose.covariance[7] = 0.0001
                odom.pose.covariance[35] = 0.01
                odom.twist.covariance[0] = 0.00001
                odom.twist.covariance[7] = 0.00001
                odom.twist.covariance[35] = 0.01

                self.odom_pub.publish(odom)
            
            elif values[0]== "ACC":
                self.accel = [ float(values[1]), float(values[2]), float(values[3])]
            
            elif values[0]== "GYR":
                self.gyro = [ float(values[1]), float(values[2]), float(values[3])]

            elif values[0] == "IMQ":
                self.imu_quat = [float(values[1]),float(values[2]),float(values[3]),float(values[4])]

            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"
            #imu_msg.orientation.w = self.imu_quat[0]
            #imu_msg.orientation.x = self.imu_quat[1]
            #imu_msg.orientation.y = self.imu_quat[2]
            #imu_msg.orientation.z = self.imu_quat[3]
            #imu_msg.angular_velocity.x = self.gyro[0] 
            #imu_msg.angular_velocity.y = self.gyro[1] 
            #imu_msg.angular_velocity.z = self.gyro[2] 
            #imu_msg.linear_acceleration.x = self.accel[0]* 9.81
            #imu_msg.linear_acceleration.y = self.accel[1]* 9.81
            #imu_msg.linear_acceleration.z = self.accel[2] * 9.81

            self.imu_pub.publish(imu_msg)    

    def vel_callback(self,vel):
   
        self.v = vel.linear.x
        self.w = vel.angular.z
        L = self.wheelbase
        R = self.wheel_radius
        
        #convert unicycle to diff drive
        v_l = ((2.0*self.v)-(self.w*L))/(2.0*R)
        v_r = ((2.0*self.v)+(self.w*L))/(2.0*R)
        
        self.wheel_speed_setpoint(v_r,v_l)
        
    def convert_vel_RPM(self,v):
        v_mag = v
        power = v/(2*3.14159)*60
        if power > self.rpm_max:
             power = self.rpm_max
        else:
            power = power
        
        return power
    
    def wheel_speed_setpoint(self,vr,vl):

        l_data = "0000"
        r_data = "0000" 
        rpm_r = int(self.convert_vel_RPM(vr))
        rpm_l = int(self.convert_vel_RPM(vl))


        r_data = '{:04d}'.format(abs(rpm_r))
        l_data = '{:04d}'.format(abs(rpm_l))

        dir_l = "0"
        dir_r = "0"

        if rpm_l != 0:
            if (rpm_l / abs(rpm_l)) >= 0 :
                dir_l = "0"
            
            elif (rpm_l / abs(rpm_l)) < 0 :
                dir_l = "1"
        
        
        if rpm_r != 0:
            if (rpm_r / abs(rpm_r)) >= 0 :
                dir_r = "0"
            
            elif (rpm_r / abs(rpm_r)) < 0 :
                dir_r = "1"


        sp_cmd = "MSP"+l_data + "," + r_data+"\n"
        self.arduino.write(sp_cmd.encode("utf-8"))

        dir_cmd = "MDR"+dir_l + "," + dir_r + "\n"
        self.arduino.write(dir_cmd.encode("utf-8"))
        print(dir_cmd)
        print(sp_cmd)

def main(args=None):
    rospy.init_node('Robot_core', anonymous=True)
    robot = Robot()
    robot.rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
