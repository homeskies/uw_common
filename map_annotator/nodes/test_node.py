#!/usr/bin/env python

import rospy
import knowledge_representation
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


def wait_for_time():
    """
        Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def database_demo():
    ltmc = knowledge_representation.get_default_ltmc()
    # retrieve a map of demo_map, or create one called demo_map if no such map exists.
    demo_map = ltmc.get_map("demo_map")

    if demo_map.get_point("point1") is None:
        demo_map.add_point("point1", 1.0, 2.0)
    pt2 = demo_map.get_point("point2")
    if not pt2:
        demo_map.add_point("point2", 2.0, 3.0)

    print("In demo_map:")
    print("POINTS: ")
    points = demo_map.get_all_points()
    for pt in points:
        print(pt.get_name())
        print("\t" + str(pt.x) + ", " + str(pt.y))

    if not demo_map.get_pose("pose1"):
        demo_map.add_pose("pose1", 1.0, 2.0, 3.14)
    if not demo_map.get_pose("pose2"):
        demo_map.add_pose("pose2", 2.0, 3.0, 0)

    print("POSES: ")
    poses = demo_map.get_all_poses()
    for ps in poses:
        print(ps.get_name())
        print("\t" + str(ps.x) + ", " + str(ps.y) + ", " + str(ps.theta))


    if not demo_map.get_region("region1"):
        demo_map.add_region("region1", [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)])
    if not demo_map.get_region("region2"):
        demo_map.add_region("region2", [(10.0, 20.0), (30.0, 40.0)])
    if not demo_map.get_region("region3"):
        demo_map.add_region("region3", [(100.0, 200.0), (300.0, 400.0), (500.0, 600.0)])

    print("REGIONS: ")
    regions = demo_map.get_all_regions()
    for r in regions:
        print(r.get_name())
        for pt in r.points:
            print("\t" + str(pt))

    print("******************************************")
    print("Get all maps:")
    demo_map1 = ltmc.get_map("demo_map1")
    map_concept = ltmc.get_concept("map")
    all_map_instances = map_concept.get_instances()
    for m in all_map_instances:
        print(m.get_name())

    print("******************************************")
    print("Map Rename")
    print("before: " + str(demo_map.get_name()))
    demo_map.rename("demo_map_renamed")
    print("after: " + str(demo_map.get_name()))


    # clean up
    demo_map.delete()
    demo_map1.delete()


def coordinate_conversion_demo():
    # pixel coordinate to map cpprdinate conversion 
    viz_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
    rospy.sleep(0.5)
    
    # homeskies_arena
    # pixel_width = 88
    # pixel_height = 113
    # resolution = 0.05
    
    # map_origin_x = -2.119012
    # map_origin_y = -3.196796

    # pixel_x = 44
    # pixel_y = 56.5
    # yaw = 0

    # uw_gazebo map
    pixel_width = 1216
    pixel_height = 768
    resolution = 0.025
    
    map_origin_x = -10
    map_origin_y = -10

    pixel_x = 640
    pixel_y = 426
    yaw = -1.2 # radians

    map_origin_y = (pixel_height - abs(map_origin_y) / resolution) * resolution
    map_x = pixel_x * resolution - abs(map_origin_x)
    map_y = - (pixel_y * resolution - abs(map_origin_y))
    map_quaternion = quaternion_from_euler(0, 0, yaw)

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = rospy.Time.now()
    ps.pose.position = Point(map_x, map_y, 0)
    ps.pose.orientation = Quaternion(map_quaternion[0], map_quaternion[1], map_quaternion[2], map_quaternion[3])
    marker = Marker(type=Marker.ARROW,
                    id=2,
                    pose=ps.pose,
                    scale=Vector3(0.5, 0.1, 0.1),
                    header=ps.header,
                    color=ColorRGBA(1.0, 0.75, 0.3, 1.0))
    viz_pub.publish(marker)


def main():
    rospy.init_node('test_node')
    wait_for_time()

    database_demo()


if __name__ == '__main__':
    main()	