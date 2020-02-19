#! /usr/bin/env python

import rospy
import math
from pprint import pprint
from access_teleop_msgs.msg import DeltaPX, PX, PXAndTheta, Theta, HeadZ
import fetch_api
import camera_info_messages
from std_msgs.msg import String, Header, ColorRGBA, Bool
from moveit_commander import MoveGroupCommander
from image_geometry import PinholeCameraModel
from tf import TransformBroadcaster, transformations
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
# from shared_teleop_functions_and_vars import wait_for_time, quat_array_to_quat, publish_camera_transforms, publish_camera_info, \
#     publish_gripper_pixels, dpx_to_distance, delta_modified_stamped_pose, \
#     add_marker, addSetback, orientation_mapping, orientation_sign_mapping, camera_names



camera_info_mapping = {'camera1': camera_info_messages.camera1, 'camera2': camera_info_messages.camera2, 'camera3': camera_info_messages.camera3}

# Xinyi added: Change the camera angle here
# original:
# transform_broadcaster_mapping = {'camera1': ((0.5, -0.3, 2.6), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
#                                 'camera2': ((0.7, -2.0, 0.55), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link')}
transform_broadcaster_mapping = {
        'camera1': ((0.7, 0, 2.3), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
        'camera2': ((0.7, -2.0, 0.8), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link'),
        'camera3': ((1.7, 0, 0.7), (0.5, 0.5, -0.5, -0.5), rospy.Time(10), 'camera3', 'base_link')
        }

# # with shelf:
# transform_broadcaster_mapping = {
#         'camera1': ((0.7, 0, 2.3), (1, 0, 0, 0), rospy.Time(10), 'camera1', 'base_link'),
#         'camera2': ((0.9, -1.2, 1.1), (-0.70711, 0, 0, 0.70711), rospy.Time(10), 'camera2', 'base_link'),
#         'camera3': ((1.7, -0.1, 1.1), (0.5, 0.5, -0.5, -0.5), rospy.Time(10), 'camera3', 'base_link')
#         }
orientation_mapping = {'camera1': 2, 'camera2': 1, 'camera3': 1}
orientation_sign_mapping = {'camera1': -1, 'camera2': 1, 'camera3': 1}
camera_names = ['camera1', 'camera2', 'camera3']


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def quat_array_to_quat(quat_array):
    new_quat = Quaternion()
    new_quat.x = quat_array[0]
    new_quat.y = quat_array[1]
    new_quat.z = quat_array[2]
    new_quat.w = quat_array[3]
    return new_quat


def publish_camera_transforms(tb, pub):
    id = 0
    for key in transform_broadcaster_mapping:
        transform_data = transform_broadcaster_mapping[key]
        tb.sendTransform(transform_data[0], transform_data[1], transform_data[2], transform_data[3], transform_data[4])
        # Debug: visualize camera poses
        marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=id,
                pose=Pose(Point(transform_data[0][0], transform_data[0][1], transform_data[0][2]), 
                          Quaternion(transform_data[1][0], transform_data[1][1], transform_data[1][2], transform_data[1][3])),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=transform_data[3])
        pub.publish(marker)
        id += 1
        

def publish_camera_info(publishers):
    for pub in publishers:
        pub[1].publish(camera_info_mapping[pub[0]])


def publish_gripper_pixels(camera_model, move_group, pub):
    data_array = []

    ps = move_group.get_current_pose()

    for camera in camera_names:
        camera_model.fromCameraInfo(camera_info_mapping[camera])
        x, y, z = getCameraDistances(camera, ps)
        (u, v) = camera_model.project3dToPixel((x, y, z))
        data_array.append([camera, int(u), int(v)])

    for array in data_array:
        px_msg = PX()
        px_msg.camera_name = array[0]
        px_msg.pixel_x = array[1]
        px_msg.pixel_y = array[2]
        pub.publish(px_msg)


def getCameraDistances(camera_name, ps):
    if camera_name == "camera1":
        camera_location = transform_broadcaster_mapping["camera1"][0]
        z = camera_location[2] - ps.pose.position.z
        x = ps.pose.position.x - camera_location[0]
        y = camera_location[1] - ps.pose.position.y
    elif camera_name == "camera2":
        camera_location = transform_broadcaster_mapping["camera2"][0]
        z = camera_location[1] - ps.pose.position.y
        x = camera_location[0] - ps.pose.position.x
        y = camera_location[2] - ps.pose.position.z
    elif camera_name == "camera3":
        camera_location = transform_broadcaster_mapping["camera3"][0]
        z = camera_location[1] - ps.pose.position.y
        x = camera_location[0] - ps.pose.position.x
        y = camera_location[2] - ps.pose.position.z
    else:
        raise ValueError('Did not pass in a valid camera_name')
    return x,y,z


def dpx_to_distance(dx, dy, camera_name, current_ps, offset):
    print("The dx is " + str(dx) + " the dy is " + str(dy) + " and the camera name is " + camera_name)

    big_z_mappings = {'camera1': transform_broadcaster_mapping['camera1'][0][2] - current_ps.pose.position.z, # x-y plane
                      'camera2': transform_broadcaster_mapping['camera2'][0][1] - current_ps.pose.position.y, # x-z plane
                      'camera3': transform_broadcaster_mapping['camera3'][0][0] - current_ps.pose.position.x} # y-z plane

    #print("The frame_id for the current pose is " + current_ps.header.frame_id)
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info_mapping[camera_name])
    x, y, z = camera_model.projectPixelTo3dRay((dx, dy))
    #print " x : {} , y : {} , z : {}".format(x, y, z)
    x_center, y_center, z_center = camera_model.projectPixelTo3dRay((0, 0))
    big_z = abs(big_z_mappings[camera_name])
    #print("The big_z for " + camera_name + " is " + str(big_z))
    big_x = (x / z) * big_z  # Use law of similar trianges to solve
    big_y = (y / z) * big_z

    big_x_center = (x_center / z_center) * big_z
    big_y_center = (y_center / z_center) * big_z

    #print("The x_center  is " + str(x_center) + " the y_center  is " + str(y_center) + " and the z_center is " + str(
    #    z_center))
    #print(
    #"The x distance is " + str(big_x - big_x_center) + " the y distance is " + str(big_y - big_y_center) + " and the camera name is " + camera_name + "\n")
    if offset:
        return big_x - big_x_center, big_y - big_y_center
    else:
        return big_x, big_y


def delta_modified_stamped_pose(x_distance, y_distance, camera_name, original_pose_stamped):
    modified_ps = original_pose_stamped
    if camera_name == 'camera1':
        modified_ps.pose.position.x += x_distance  # These directions came from looking at the cameras in rviz
        modified_ps.pose.position.y -= y_distance
    elif camera_name == 'camera2':
        modified_ps.pose.position.x += x_distance
        modified_ps.pose.position.z -= y_distance
    elif camera_name == 'camera3':
        modified_ps.pose.position.y += x_distance
        modified_ps.pose.position.z -= y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')
    return modified_ps


def absolute_modified_stamped_pose(x_distance, y_distance, camera_name, original_pose_stamped):
    modified_ps = original_pose_stamped
    if camera_name == 'camera1':
        modified_ps.pose.position.x = transform_broadcaster_mapping['camera1'][0][0] + x_distance
        modified_ps.pose.position.y = transform_broadcaster_mapping['camera1'][0][1] - y_distance
    elif camera_name == 'camera2':
        modified_ps.pose.position.x = transform_broadcaster_mapping['camera2'][0][0] + x_distance
        modified_ps.pose.position.z = transform_broadcaster_mapping['camera2'][0][2] - y_distance
    elif camera_name == 'camera3':
        modified_ps.pose.position.y = transform_broadcaster_mapping['camera3'][0][1] + x_distance
        modified_ps.pose.position.z = transform_broadcaster_mapping['camera3'][0][2] - y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')
    return modified_ps


def add_marker(x_distance, y_distance, ps, camera_name, self):
    controls = InteractiveMarkerControl()

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.05
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    int_marker = InteractiveMarker()
    int_marker.header = ps.header
    int_marker.name = "click_location"

    if camera_name == 'camera1':
        int_marker.pose.position.z = ps.pose.position.z
        int_marker.pose.position.x = transform_broadcaster_mapping['camera1'][0][0] + x_distance
        int_marker.pose.position.y = transform_broadcaster_mapping['camera1'][0][1] - y_distance
    elif camera_name == 'camera2':
        int_marker.pose.position.y = ps.pose.position.y
        int_marker.pose.position.x = transform_broadcaster_mapping['camera2'][0][0] + x_distance
        int_marker.pose.position.z = transform_broadcaster_mapping['camera2'][0][2] - y_distance
    elif camera_name == 'camera3':
        int_marker.pose.position.x = ps.pose.position.x
        int_marker.pose.position.y = transform_broadcaster_mapping['camera3'][0][1] + x_distance
        int_marker.pose.position.z = transform_broadcaster_mapping['camera3'][0][2] - y_distance
    else:
        raise ValueError('Did not pass in a valid camera_name')

    int_marker.pose.orientation.w = 1
    controls.markers.append(box_marker)
    int_marker.controls.append(controls)
    self._im_server.insert(int_marker)
    self._im_server.applyChanges()
    print("Marker changes should now be applied")


def addSetback(setback, theta, camera_name, ps):
    modified_ps = ps
    delta_y = math.sin(theta) * setback
    delta_x = math.cos(theta) * setback
    if camera_name == 'camera1':   # x-y plane
        modified_ps.pose.position.x -= delta_x
        modified_ps.pose.position.y += delta_y
    elif camera_name == 'camera2': # x-z plane
        modified_ps.pose.position.x -= delta_x
        modified_ps.pose.position.z += delta_y
    elif camera_name == 'camera3': # y-z plane
        modified_ps.pose.position.y -= delta_x
        modified_ps.pose.position.z += delta_y
    else:
        raise ValueError('Did not pass in a valid camera_name')


    return modified_ps

###################################################################################################

class MoveByDelta(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/delta', DeltaPX, self.callback, queue_size=1)

    def callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.delta_x, data.delta_y, data.camera_name, ps, True)
        ps2 = delta_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        pose_possible = self._arm.compute_ik(ps2, timeout=rospy.Duration(1))
        print(pose_possible)
        if pose_possible:  # This check will prevent some edge poses, but will also save time
            error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
            if error is not None:
                rospy.logerr(error)
            else:
                print("We got there!")


class MoveByAbsolute(object):

    def __init__(self, arm, move_group, status_pub):
        self._arm = arm
        self._move_group = move_group
        self._im_server = InteractiveMarkerServer('im_server', q_size=2)
        self._status_pub = status_pub

    def start(self):
        rospy.Subscriber('/access_teleop/absolute', PX, self.absolute_callback, queue_size=1)

    def absolute_callback(self, data):
        ps = self._move_group.get_current_pose()
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        # add_marker(x_distance, y_distance, ps, data.camera_name, self)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        pose_possible = self._arm.compute_ik(ps2, timeout=rospy.Duration(1))
        print(pose_possible)
        if pose_possible:  # This check will prevent some edge poses, but will also save time
            self._status_pub.publish("moving")
            error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
            if error is not None:
                rospy.logerr(error)
            else:
                self._status_pub.publish("arrived")
        else:
            self._status_pub.publish("unreachable")


class MoveAndOrient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/move_and_orient', PXAndTheta, self.move_and_orient_callback, queue_size=1)

    def move_and_orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        # rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] = data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        error = self._arm.move_to_pose(ps2, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class MoveAndOrient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group
        self.SETBACK = 0.15

    def start(self):
        rospy.Subscriber('/access_teleop/move_and_orient', PXAndTheta, self.move_and_orient_callback, queue_size=1)

    def move_and_orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        # rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] = data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        x_distance, y_distance = dpx_to_distance(data.pixel_x, data.pixel_y, data.camera_name, ps, False)
        ps2 = absolute_modified_stamped_pose(x_distance, y_distance, data.camera_name, ps)
        ps3 = addSetback(self.SETBACK, data.theta, data.camera_name, ps2)
        error = self._arm.move_to_pose(ps3, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class Orient(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/orient', Theta, self.orient_callback, queue_size=1)

    def orient_callback(self, data):
        ps = self._move_group.get_current_pose()
        rpy = self._move_group.get_current_rpy()
        # rpy = [0,0,0]
        print("The curent orientation for that camera is")
        pprint(rpy)
        rpy[orientation_mapping[data.camera_name]] += data.theta * orientation_sign_mapping[data.camera_name]
        print("The new orientation of the gripper is ")
        pprint(rpy)
        new_quat_array = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], "sxyz")
        ps.pose.orientation = quat_array_to_quat(new_quat_array)
        error = self._arm.move_to_pose(ps, allowed_planning_time=1.0)
        if error is not None:
            rospy.logerr(error)


class WristRoll(object):
    def __init__(self, arm, move_group):
        self._arm = arm
        self._move_group = move_group

    def start(self):
        rospy.Subscriber('/access_teleop/wrist_roll', Theta, self.wrist_roll_callback, queue_size=1)

    def wrist_roll_callback(self, data):
        self._move_group.clear_pose_targets()
        arm_values = self._move_group.get_current_joint_values()
        arm_values[6] += data.theta
        self._move_group.set_joint_value_target(arm_values)
        self._move_group.go()


# Added by Xinyi for tilting robot's head
class HeadTilt(object):
    def __init__(self, head):
        self._head = head
        self._current_head_tilt = 0  # the current head tilt


    def start(self):
        rospy.Subscriber('/access_teleop/head_z', HeadZ, self.head_tilt_callback, queue_size=1)

    def head_tilt_callback(self, data):
        self._current_head_tilt -= data.tilt
        self._head.pan_tilt(0, self._current_head_tilt)



def main():
    rospy.init_node('access_teleop')
    wait_for_time()

    # Added by Xinyi:
    # Fetch controls
    torso = fetch_api.Torso()
    arm = fetch_api.Arm()
    base = fetch_api.Base()
    head = fetch_api.Head()

    move_group = MoveGroupCommander("arm")

    status_publisher = rospy.Publisher('/access_teleop/arm_status', String, queue_size=1)
    gripper_publisher = rospy.Publisher('/access_teleop/gripper_pixels', PX, queue_size=1)

    info_pubs = []
    for camera_name in camera_names:
        info_pubs.append([camera_name,
                          rospy.Publisher(camera_name + '/camera_info', camera_info_messages.CameraInfo, queue_size=1)])

    # Added by Xinyi
    # Debug: visualize camera positions
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)
    # (end)

    tb = TransformBroadcaster()

    camera_model = PinholeCameraModel()

    move_by_delta = MoveByDelta(arm, move_group)
    move_by_delta.start()

    move_by_absolute = MoveByAbsolute(arm, move_group, status_publisher)
    move_by_absolute.start()

    move_and_orient = MoveAndOrient(arm, move_group)
    move_and_orient.start()

    orient = Orient(arm, move_group)
    orient.start()

    # Added by Xinyi
    wrist_roll = WristRoll(arm, move_group)
    wrist_roll.start()

    head_tilt = HeadTilt(head)
    head_tilt.start()

    rospy.sleep(0.5)
    # (end)

    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        publish_camera_transforms(tb, vis_pub)
        publish_camera_info(info_pubs)
        publish_gripper_pixels(camera_model, move_group, gripper_publisher)

        rate.sleep()


if __name__ == "__main__":
    main()
