#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from firebase import firebase as fb
import os


class Listener:
    def __init__(self, topics):
        rospy.init_node('firebase', anonymous=True)
        self.firebase = fb.FirebaseApplication(
            'https://<firebase-url>.firebaseio.com/', authentication=None)

        for topic in topics:
            rospy.Subscriber(
                topic, String, callback=self.callback, callback_args=topic)

        rospy.spin()

    def callback(self, msg, topic):
        data = {'timestamp': rospy.get_time(), 'msg': msg.data}
        print ' <' + topic + '> ' + msg.data
        self.firebase.post(topic, data)


if __name__ == '__main__':
    filename = os.path.join(os.path.dirname(__file__), 'topics.txt')
    topics = list(open(filename, 'r'))
    l = Listener(topics)
