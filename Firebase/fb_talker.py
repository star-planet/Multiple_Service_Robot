#! /usr/bin/env python

import time

# ROS
import rospy
from std_msgs.msg import Int16

# Firebase
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

print(
    '''
ROS & Firebase communication
------------------------------
    '''
)
    
rospy.init_node("firebase_to_ros", anonymous=True)
pub = rospy.Publisher("test", Int16, queue_size=10)

cred = credentials.Certificate('mca_key.json') # Key Certification

firebase_admin.initialize_app(cred, {
    'databaseURL': '".firebaseio.com/'
})
    
while not rospy.is_shutdown():
    ref = db.reference('test/msg')
    data = ref.get()
    
    if not data == None:
        print("Firebase TO ROS: " + str(data))
        pub.publish(int(data))
        time.sleep(1)
        ref.delete()
