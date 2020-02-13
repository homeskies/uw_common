#!/usr/bin/env python
from __future__ import print_function
import rospy
import httplib2
from std_msgs.msg import String
import speech_recognition as sr
from speech_recognition import Recognizer, Microphone


class Speech(object):

    def __init__(self):
        rospy.init_node("speech")
        self.pub = rospy.Publisher("speech_recognizer", String, latch=True, queue_size=1)
        self.recognizer = Recognizer()
        # self.recognizer.energy_threshold = 1000
        # self.recognizer.pause_threshold = .7
        self.microphone = Microphone()

    @staticmethod
    def check_internet_connection():
        connection = httplib2.HTTPConnectionWithTimeout("www.google.com", timeout=5)
        try:
            connection.request("HEAD", "/")
            connection.close()
            return True
        except httplib2.HttpLib2Error:
            connection.close()
            return False

    def __recognize_helper(self, recognized_speech):
        '''
        Once speech obtained do TODO:something with it
        '''
        print(recognized_speech)
        if recognized_speech != "":
            rospy.loginfo("You said: " + recognized_speech)
            self.pub.publish(recognized_speech)

    def __microphone_helper(self, ):
        rospy.loginfo('Listening...')
        with self.microphone as source:
            # If phase_time_limit is not set to 5 it will
            # take a really long time every 3-4th attempt
            audio = self.recognizer.listen(source, phrase_time_limit=5)
        rospy.loginfo('Got a sound; recognizing...')
        return audio

    def recognize_google(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        try:
            while not rospy.is_shutdown():
                audio = self.__microphone_helper()
                recognized_speech = ""
                if Speech.check_internet_connection():
                    try:
                        recognized_speech = self.recognizer.recognize_google(audio)
                    except sr.UnknownValueError:
                        rospy.logerr("Could not understand audio.")
                    except sr.RequestError:
                        rospy.logerr("Could not request results.")
                else:
                    rospy.logerr("No internet conneciton for Google API")

                self.__recognize_helper(recognized_speech)

        except Exception as exc:
            rospy.logerr(exc)

    def recognize_sphynix(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        try:
            while not rospy.is_shutdown():
                audio = self.__microphone_helper()
                recognized_speech = ""

                try:
                    recognized_speech = self.recognizer.recognize_sphinx(audio)
                except sr.UnknownValueError:
                    rospy.logerr("Could not understand audio.")
                except sr.RequestError:
                    rospy.logerr("""Could not request results. Do you have
                     pocket Sphynx installed?""")
                self.__recognize_helper(recognized_speech)
        except Exception as exc:
            rospy.logerr(exc)
