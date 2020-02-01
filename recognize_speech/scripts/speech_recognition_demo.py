#!/usr/bin/env python

# from speech import Speech
from speech import Speech

def main():
    speaker = Speech()
    speaker.recongizeGoogle()
    rospy.spin()

if __name__ == '__main__':
    main()
    