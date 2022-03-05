#!/usr/bin/python
from cmath import cos
import rospy 
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import tf
import matplotlib.pyplot as plt
import pandas as pd
import math

import csv
import numpy
import statistics

class Plot:
    def __init__(self):
        rospy.init_node('ntt_west_ref_plot', anonymous=True)

        rospy.Subscriber('initialpose', PoseWithCovarianceStamped,  lambda x: self.pose_callback(x))
        
        self.ekf_diff_=[]
        self.ndt_diff_=[]

        self.df_ekf_=pd.read_csv('your_ndt_pose.csv')
        self.df_ndt_=pd.read_csv('your_ekf_pose.csv')
        
        

    def pose_callback(self, pose_msg):
        self.ref_pose_=pose_msg

        min_ex=0.2
        min_ey=0.2
        for i in range(len(self.df_ekf_['x'])):
            k=numpy.sqrt((pose_msg.pose.pose.position.x-self.df_ekf_['x'].iat[i])**2+(pose_msg.pose.pose.position.y-self.df_ekf_['y'].iat[i])**2)
            if k<0.2:
                x=self.df_ekf_['x'].iat[i]-pose_msg.pose.pose.position.x
                y=self.df_ekf_['y'].iat[i]-pose_msg.pose.pose.position.y
                e = tf.transformations.euler_from_quaternion((pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w))
                x2=math.cos(e[2])*x + math.sin(e[2])*y
                y2=math.cos(e[2])*x - math.sin(e[2])*y
                if min_ex>abs(x2):
                    min_ex=abs(x2)
                    min_ey=y2
        

        min_nx=0.2
        min_ny=0.2
        for i in range(len(self.df_ndt_['x'])):
            k=numpy.sqrt((pose_msg.pose.pose.position.x-self.df_ndt_['x'].iat[i])**2+(pose_msg.pose.pose.position.y-self.df_ndt_['y'].iat[i])**2)
            if k<0.2:
                x=self.df_ndt_['x'].iat[i]-pose_msg.pose.pose.position.x
                y=self.df_ndt_['y'].iat[i]-pose_msg.pose.pose.position.y
                e = tf.transformations.euler_from_quaternion((pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w))
                x2=math.cos(e[2])*x + math.sin(e[2])*y
                y2=math.cos(e[2])*x - math.sin(e[2])*y
                if min_nx>abs(x2):
                    min_nx=abs(x2)
                    min_ny=y2
        

        if min_ny==0.2 or min_ey==0.2:
            print("try again")
        else:
            self.ekf_diff_.append(float(min_ey))
            self.ndt_diff_.append(float(min_ny))
        
            print("calc end")
            print(min_ny)
            print(min_ey)

        
    
if __name__ == '__main__':
    c=Plot()
    rospy.spin()

    mean_ekf=statistics.mean(c.ekf_diff_)
    mean_ndt=statistics.mean(c.ndt_diff_)
    pvarianceekf = statistics.pvariance(c.ekf_diff_)
    pvariancendt = statistics.pvariance(c.ndt_diff_)
    print("mean_ekf="+str(mean_ekf))
    print("mean_ndt="+str(mean_ndt))
    print("pvariance_ekf="+str(pvarianceekf))
    print("pvariance_ndt="+str(pvariancendt))
    print("pstdev_ekf="+str(math.sqrt(pvarianceekf)))
    print("pstdev_ndt="+str(math.sqrt(pvariancendt)))
    


    with open('your_diff.csv', 'w')  as fe:
        writer=csv.writer(fe)
        writer.writerow(['ndt_diff', 'ekf_diff'])
        for i in range(len(c.ekf_diff_)):
            writer.writerow([c.ndt_diff_[i],c.ekf_diff_[i]])
        
    