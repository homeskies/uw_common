#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import httplib2 as httplib
from std_msgs.msg import String
import speech_recognition as sr

class Speech(object):

    def __init__(self):
        rospy.init_node("speech_recognizer")
        self.pub = rospy.Publisher("speech_recognizer", String, latch=True, queue_size=1)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

    @staticmethod
    def check_internet_connection():
        connection = httplib.HTTPConnection("www.google.com", timeout=5)
        try:
            connection.request("HEAD", "/")
            connection.close()
            return True
        except:
            connection.close()
            return False

    def recognizeHelper(recognized_speech):
        '''
        Once speech obtained do TODO:something with it
        '''
        if recognized_speech != "":
            rospy.loginfo("You said: " + recognized_speech)
            self.pub.publish(recognized_speech)

    def recognizeGoogle(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        try:
            while not rospy.is_shutdown():
                rospy.loginfo('Listening...')
                with self.microphone as source:
                    audio = self.recognizer.listen(source)
                rospy.loginfo('Got a sound; recognizing...')

                recognized_speech = ""
                if SpeechRecognizer.check_internet_connection():
                    try:
                        recognized_speech = self.recognizer.recognize_google(audio)
                    except sr.UnknownValueError:
                        rospy.logerr("Could not understand audio.")
                    except sr.RequestError:
                        rospy.logerr("Could not request results.")
                else:
                    rospy.logerr("No internet conneciton for Google API")

                recognizeHelper(recognized_speech)

        except Exception as exc:
            rospy.logerr(exc)

    def recognizeSphynix(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        try:
            while not rospy.is_shutdown():
                rospy.loginfo('Listening...')
                with self.microphone as source:
                    audio = self.recognizer.listen(source)
                rospy.loginfo('Got a sound; recognizing...')

                recognized_speech = ""

                try:
                    recognized_speech = self.recognizer.recognize_sphinx(audio)
                except sr.UnknownValueError:
                    rospy.logerr("Could not understand audio.")
                except sr.RequestError:
                    rospy.logerr("Could not request results.")
        except Exception as exc:
            rospy.logerr(exc)
        recognizeHelper(recognized_speech)