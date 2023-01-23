#! /usr/bin/env python

# ROS
import rospy
from std_msgs.msg import Int16

# Firebase
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

class Listener:
    def __init__(self):
        rospy.init_node('ros_to_firebase', anonymous=True)
        rospy.Subscriber("test", Int16, callback=self.callback)
        
        self.cred = credentials.Certificate('mca_key.json') # Key Certification

        firebase_admin.initialize_app(self.cred, {
            'databaseURL': '"URL".firebaseio.com/'
        })
        
        rospy.spin()

    def callback(self, msg):
        data = {'msg' : msg.data}
        ref = db.reference('test')
        ref.update(data)
        print("ROS TO Firebase: " + str(msg.data))

if __name__ == '__main__':
    print(
    '''
ROS & Firebase communication
------------------------------
    '''
    )
    l = Listener()
