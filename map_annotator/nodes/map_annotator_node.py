#!/usr/bin/env python

import rospy
from map_annotator_msgs.srv import GetMaps, GetMapsResponse, DeleteMap, DeleteMapResponse
from map_annotator_msgs.msg import MapAnnotation, Point, Pose, Region
import knowledge_representation

def wait_for_time():
    """
        Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class Server(object):
    def __init__(self):
        self.db = knowledge_representation.get_default_ltmc()
        self.get_maps_srv = rospy.Service("map_annotator/get_maps", GetMaps, self.get_all_maps)
        self.delete_map_srv = rospy.Service("map_annotator/delete_map", DeleteMap, self.delete_map)


    def get_all_maps(self, request):
        """ Return the list of all map names in the database. """
        map_concept = self.db.get_concept("map")
        all_map_instances = map_concept.get_instances()
        map_list = []
        for m in all_map_instances:
            map_list.append(m.get_name())
        return GetMapsResponse(map_list)


    def delete_map(self, request):
        """ Delete the requested map from database. """
        print("Map named \"" + request.name + "\" is deleted.")
        self.db.get_map(request.name).delete()
        return DeleteMapResponse()


    def process_changes(self, msg):
        print("Previous Map Name: " + msg.prev_name)
        print("Current Map Name: " + msg.current_name)
        
        target_map = None
        if msg.prev_name != "":  # a rename happens
            # TODO: rename the map
            pass

        # retrieve the map, or create one if no such map exists
        target_map = self.db.get_map(msg.current_name)
        
        # handle point changes
        self.process_point_changes(target_map, msg.points)
        # handle pose changes
        self.process_pose_changes(target_map, msg.poses)  
        # handle region changes
        self.process_region_changes(target_map, msg.regions)
    

    def process_point_changes(self, target_map, point_changes):
        print("BEFORE: ")
        self._print_all_points(target_map)

        for pt in point_changes:
            prev_pt = target_map.get_point(pt.prev_name)
            if prev_pt:
                point_x = pt.x if pt.x != None else prev_pt.x
                point_y = pt.y if pt.y != None else prev_pt.y
                # add a new point
                target_map.add_point(pt.current_name, point_x, point_y)
                # delete the existing point
                prev_pt.delete()
            else:
                curr_pt = target_map.get_point(pt.current_name)
                if curr_pt:
                    curr_pt.delete()
                target_map.add_point(pt.current_name, pt.x, pt.y)
        
        print("AFTER: ")
        self._print_all_points(target_map)

    
    def process_pose_changes(self, target_map, pose_changes):
        print("BEFORE: ")
        self._print_all_poses(target_map)

        for ps in pose_changes:
            prev_ps = target_map.get_pose(ps.prev_name)
            if prev_ps:
                pose_x = ps.x if ps.x != None else prev_ps.x
                pose_y = ps.y if ps.y != None else prev_ps.y
                pose_theta = ps.theta if ps.theta != None else prev_ps.theta
                # add a new pose
                target_map.add_pose(ps.current_name, pose_x, pose_y, pose_theta)
                # delete the existing pose
                prev_ps.delete()
            else:
                curr_ps = target_map.get_pose(ps.current_name)
                if curr_ps:
                    curr_ps.delete()
                target_map.add_pose(ps.current_name, ps.x, ps.y, ps.theta)
        
        print("AFTER: ")
        self._print_all_poses(target_map)
        
        
    def process_region_changes(self, target_map, region_changes):
        print("BEFORE: ")
        self._print_all_regions(target_map)

        for r in region_changes:
            prev_r = target_map.get_region(r.prev_name)
            if prev_r:
                pass
                # region_endpoints = ps.x if ps.x != None else prev_ps.x
                # pose_y = ps.y if ps.y != None else prev_ps.y
                # pose_theta = ps.theta if ps.theta != None else prev_ps.theta
                # # add a new pose
                # target_map.add_pose(ps.current_name, pose_x, pose_y, pose_theta)
                # # delete the existing pose
                # prev_ps.delete()
            else:
                pass
                # curr_ps = target_map.get_pose(ps.current_name)
                # if curr_ps:
                #     curr_ps.delete()
                # target_map.add_pose(ps.current_name, ps.x, ps.y, ps.theta)        
        
        print("AFTER: ")
        self._print_all_regions(target_map)

    
    def _print_all_points(self, target_map):
        print("Points: ")
        points = target_map.get_all_points()
        for pt in points:
            print(pt.get_name())
            print("\t" + str(pt.x) + ", " + str(pt.y))
        print("-------------")

    
    def _print_all_poses(self, target_map):
        print("Poses: ")
        poses = target_map.get_all_poses()
        for ps in poses:
            print(ps.get_name())
            print("\t" + str(ps.x) + ", " + str(ps.y) + ", " + str(ps.theta))
        print("-------------")


    def _print_all_regions(self, target_map):
        print("Regions: ")
        regions = target_map.get_all_regions()
        for r in regions:
            print(r.get_name())
            for pt in r.points:
                print("\t" + str(pt))
        print("-------------")


def main():
    rospy.init_node('map_annotator_node')
    wait_for_time()
    
    server = Server()

    annotation_changes_sub = rospy.Subscriber("map_annotator/changes", MapAnnotation, callback=server.process_changes)
    rospy.sleep(1)
    print("Map annotator node is running...")

    def handle_shutdown():
        print("Shutting down...")

    rospy.on_shutdown(handle_shutdown)

    rospy.spin()


if __name__ == '__main__':
    main()	