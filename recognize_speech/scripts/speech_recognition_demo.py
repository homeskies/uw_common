#!/usr/bin/env python

import recognize_speech as rs
import rospy

def main():
    speech = rs.Speech()
    speech.recognizeGoogle()
    rospy.spin()

if __name__ == '__main__':
    main()
    