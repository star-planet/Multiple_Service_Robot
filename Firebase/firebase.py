#! /usr/bin/env python

import time

# ROS
import rospy
from std_msgs.msg import String

# Firebase
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

class Listener:
    def __init__(self):
        rospy.init_node('rosFirebase', anonymous=True)
        rospy.Subscriber("ros_to_firebase", String, callback=self.callback)
        self.pub = rospy.Publisher("firebase_to_ros", String, queue_size=10)
        
        self.cred = credentials.Certificate('mca_key.json') # Key Certification

        firebase_admin.initialize_app(self.cred, {
            'databaseURL': 'https://mcaproject-9e8c7-default-rtdb.firebaseio.com/'
        })
        
        self.talker()

    def callback(self, msg):
        data = {'msg' : msg.data}
        ref = db.reference('test')
        ref.update(data)
        print("ROS TO Firebase: " + msg.data)
        
    def talker(self):
        while not rospy.is_shutdown():
            ref = db.reference('test/msg')
            data = ref.get()
            
            list = ["open","close","start"]
        
            if data in list:
                print("Firebase TO ROS: " + data)
                self.pub.publish(data)
                time.sleep(1)
                ref.delete()


if __name__ == '__main__':
    print(
    '''
ROS & Firebase communication
------------------------------
    '''
    )
    l = Listener()
