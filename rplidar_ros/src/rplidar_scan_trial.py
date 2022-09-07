#!/usr/bin/env python
import rospy
import math
import numpy
import os
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan


def callback(msg):
    os.system('clear')
    print("**********************Recordings for one Object***********************")
    list_till_min=[]
    list_after_min=[]

    #Finding distance between Lidar and closest object
    min_distance = min(msg.ranges)*100   
   # print("\n")  
    print("Distance of closest object is ",min_distance," cm")

    #made by me
    pub=rospy.Publisher('min_distance', Float32, queue_size=10)
    pub.publish(min_distance)

    if min_distance>18:
        distance_result = 1
    else:
        distance_result = 0

    pub2=rospy.Publisher('distance_result', Int32, queue_size=10)
    pub2.publish(distance_result)

    print("distance_result: ", distance_result)
    
    # Finding angle corresponding to closest object
    for i in range(0,len(msg.ranges)):
        list_till_min.append(msg.ranges[i]*100)
        if min_distance==msg.ranges[i]*100: 
            break
    index_till_min=i
    angle=i*0.017    
    #print('angle values:',angle)  
    # converting angle in degrees 
    print('Angle of closest object is :',(numpy.degrees(angle))," degrees")     
  
    # To get points across length/width of an obstacle
    for j in range(index_till_min+1,len(msg.ranges)):
        list_after_min.append(msg.ranges[j]*100)

    l1_string = ', '.join(map(str, list_till_min))
    length_of_l1_string = len(l1_string)
    last_index_of_inf1 = l1_string.rfind('inf') 
    substring_till_min_values = l1_string[last_index_of_inf1+5:length_of_l1_string]
    list_till_min_values = substring_till_min_values.split(", ")

    #Converting list of strings to list of float
    for i in range(len(list_till_min_values)):
         list_till_min_values[i]=float(list_till_min_values[i])
    
    # To get points across length/width of an obstacle
    l2_string = ', '.join(map(str, list_after_min))
    length_of_l2_string = len(l2_string)
    last_index_of_inf2 = l2_string.find('inf') 
    substring_after_min_values = l2_string[0:last_index_of_inf2-5]
    list_after_min_values = substring_after_min_values.split(", ")

    #Converting list of strings to list of float
    for i in range(len(list_after_min_values)):
         list_after_min_values[i]=float(list_after_min_values[i])
   

    #print('list till minimum values: ',list_till_min_values)
    #print('list after minimun values: ',list_after_min_values)

   

    # To find angle for d1 and d2
    d1_angle=len(list_till_min_values)*0.017
    side1=pow(pow(list_till_min_values[0],2)+pow(min_distance,2)-2*list_till_min_values[0]*min_distance*math.cos(d1_angle),0.5)

    d2_angle=len(list_after_min_values)*0.017
    side2=pow(pow(list_after_min_values[len(list_after_min_values)-1],2)+pow(min_distance,2)-2*list_after_min_values[len(list_after_min_values)-1]*min_distance*math.cos(d2_angle),0.5)  

    ################# for I-Shaped Obstacle############################

    if(abs(list_till_min_values[0]-list_till_min_values[len(list_till_min_values)-1])<2 and abs(list_after_min_values[0]-list_after_min_values[len(list_after_min_values)-1])<2):
        print("I Shape Object, Length/Width is ",side1+side2," cm")
    elif(side1<side2):
        print("Width is ",side1," cm")
        print("Length is ",side2," cm")
    else:
        print("Length is ",side1," cm")
        print("Width is ",side2," cm")
        

  

    
def rplidar_scan_trial():
        rospy.init_node('rplidar_listener')  #Initializing subscriber node
        rospy.Subscriber('/scan', LaserScan, callback)
        rospy.spin()

      

if __name__ == '__main__':
    rplidar_scan_trial()
