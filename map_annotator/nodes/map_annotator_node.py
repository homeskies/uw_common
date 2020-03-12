#!/usr/bin/env python

import rospy
from map_annotator_msgs.msg import PointAnnotation, PoseAnnotation, RegionAnnotation
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

    def process_point(self, msg):
        print("POINT")
        if msg.command == "save":
            self.save_point(msg.name, msg.x, msg.y)
        elif msg.command == "delete":
            self.delete_point(msg.name)
        else:  # rename
            self.rename_point(msg.name)

    def save_point(self, name, x, y):
        if name != "":
            print("save name")
        if x != -1:
            print("save x")
        if y != -1:
            print("save y")
    
    def delete_point(self, name):
        pass

    def rename_point(self, name):
        pass



def main():
    rospy.init_node('map_annotator_node')
    wait_for_time()
    
    # ltmc = knowledge_representation.get_default_ltmc()
    database = Database()
    server = Server(database)

    point_annotation_sub = rospy.Subscriber("map_annotator/point", PointAnnotation, callback=server.process_point)
    # pose_annotation_sub = rospy.Subscriber("map_annotator/pose", PoseAnnotation, callback=server.process_pose)
    # region_annotation_sub = rospy.Subscriber("map_annotator/region", RegionAnnotation, callback=server.process_region)
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