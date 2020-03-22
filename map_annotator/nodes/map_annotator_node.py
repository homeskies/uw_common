#!/usr/bin/env python

import rospy
from map_annotator_msgs.msg import MapAnnotation, Point, Pose, Region
import knowledge_representation

def wait_for_time():
    """
        Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class Database(object):
    def __init__(self):
        pass


class Server(object):
    def __init__(self, database):
        self.db = database
        # self.load()

    def load(self):
        pass

    def process_changes(self, msg):
        print("Previous Map Name: " + msg.prev_name)
        print("Current Map Name: " + msg.current_name)
        print("POINTS: ")
        for pt in msg.points:
            print(pt)
        print("POSES: ")
        for ps in msg.poses:
            print(ps)
        print("REGIONS: ")
        for r in msg.regions:
            print(r)


def main():
    rospy.init_node('map_annotator_node')
    wait_for_time()
    
    # ltmc = knowledge_representation.get_default_ltmc()
    database = Database()
    server = Server(database)

    annotation_changes_sub = rospy.Subscriber("map_annotator/changes", MapAnnotation, callback=server.process_changes)
    rospy.sleep(1)

    def handle_shutdown():
        # poses = PoseNames()
        # server.publisher.publish(poses)
        # database.save()
        pass

    rospy.on_shutdown(handle_shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()	