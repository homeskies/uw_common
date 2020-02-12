#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse
from web_teleop.srv import OpenGripper, OpenGripperResponse, CloseGripper, CloseGripperResponse
from web_teleop.srv import SetArm, SetArmResponse
from web_teleop.srv import MoveHead, MoveHeadResponse
from fetch_api import ArmJoints 

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._gripper = fetch_api.Gripper()
        self._arm = fetch_api.Arm()
        self._head = fetch_api.Head()

    def handle_set_torso(self, request):
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def open_gripper(self, request):
        self._gripper.open()
        return OpenGripperResponse()

    def close_gripper(self, request):
        self._gripper.close(request.force)
        return CloseGripperResponse()

    def set_arm(self, request):
        j = ArmJoints.from_list(request.joints)
        self._arm.move_to_joints(j)
        return SetArmResponse()

    def move_head(self, request):
        self._head.pan_tilt(request.xv, request.yv)
        return MoveHeadResponse()



def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso, server.handle_set_torso)
    open_gripper_service = rospy.Service('web_teleop/open_gripper', OpenGripper, server.open_gripper)
    close_gripper_service = rospy.Service('web_teleop/close_gripper', CloseGripper, server.close_gripper)
    set_arm_service = rospy.Service('web_teleop/set_arm', SetArm, server.set_arm)
    move_head_service = rospy.Service('web_teleop/move_head', MoveHead, server.move_head)
    rospy.spin()


if __name__ == '__main__':
    main()