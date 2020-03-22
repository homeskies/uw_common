#!/usr/bin/env python

import rospy
import knowledge_representation


def wait_for_time():
    """
        Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('test_node')
    wait_for_time()
    
    ltmc = knowledge_representation.get_default_ltmc()
    # retrieve a map of arena, or create one called arena if no such map exists.
    arena = ltmc.get_map("arena")

    if not arena.get_point("point1"):
        arena.add_point("point1", 1.0, 2.0)
    if not arena.get_point("point2"):
        arena.add_point("point2", 2.0, 3.0)

    print("POINTS: ")
    points = arena.get_all_points()
    for pt in points:
        print(pt.get_name())
    #    print("\t" + str(pt.x) + ", " + str(pt.y))

    if not arena.get_pose("pose1"):
        arena.add_pose("pose1", 1.0, 2.0, 3.14)
    if not arena.get_pose("pose2"):
        arena.add_pose("pose2", 2.0, 3.0, 0)

    print("POSES: ")
    poses = arena.get_all_poses()
    for ps in poses:
        print(ps.get_name())
        print("\t" + str(ps.x) + ", " + str(ps.y) + ", " + str(ps.theta))

    if not arena.get_region("region1"):
        arena.add_region("region1", {{1.0, 2.0}, {3.0, 4.0}})
    if not arena.get_region("region2"):
        arena.add_region("region2", {{1.0, 2.0}, {3.0, 4.0}, {5.0, 6.0}})

    print("REGIONS: ")
    regions = arena.get_all_regions()
    for r in regions:
        print(r.get_name())
        for pt in r.points:
            print("\t" + str(r.x) + ", " + str(r.y))

if __name__ == '__main__':
    main()	