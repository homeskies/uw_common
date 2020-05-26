#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from map_annotator_msgs.srv import GetMaps, GetMapsResponse, DeleteMap, DeleteMapResponse, HasPoint, HasPointResponse, HasPose, HasPoseResponse, HasRegion, HasRegionResponse
from map_annotator_msgs.msg import MapAnnotation, Point, Pose, Region, RobotPose
import knowledge_representation
import copy

def wait_for_time():
    """
        Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class Server(object):
    def __init__(self):
        self.db = knowledge_representation.get_default_ltmc()
        # services
        self.get_maps_srv = rospy.Service("map_annotator/get_maps", GetMaps, self.get_all_maps)
        self.delete_map_srv = rospy.Service("map_annotator/delete_map", DeleteMap, self.delete_map)
        self.has_point_srv = rospy.Service("map_annotator/has_point", HasPoint, self.has_point)
        self.has_pose_srv = rospy.Service("map_annotator/has_pose", HasPose, self.has_pose)
        self.has_region_srv = rospy.Service("map_annotator/has_region", HasRegion, self.has_region)
        rospy.sleep(0.5)


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
        self.db.get_map(request.name).delete()
        print("Map named \"" + request.name + "\" is deleted.")
        return DeleteMapResponse()


    def has_point(self, request):
        """ Check if the given map has the given point. """
        result = self.db.get_map(request.map_name).get_point(request.point_name)
        if result:
            return HasPointResponse(True)
        return HasPointResponse(False)

    
    def has_pose(self, request):
        """ Check if the given map has the given pose. """
        result = self.db.get_map(request.map_name).get_pose(request.pose_name)
        if result:
            return HasPoseResponse(True)
        return HasPoseResponse(False)


    def has_region(self, request):
        """ Check if the given map has the given region. """
        result = self.db.get_map(request.map_name).get_region(request.region_name)
        if result:
            return HasRegionResponse(True)
        return HasRegionResponse(False)


    def process_changes(self, msg):
        print("**************************************")
        print("Previous Map Name: " + msg.prev_name)
        print("Current Map Name:  " + msg.current_name)
        print("--------------------------")
        
        target_map = None
        prev_map = self.has_map(msg.prev_name)
        curr_map = self.has_map(msg.current_name)

        if prev_map:
            if curr_map:
                # replace current map with previous map
                print("Map \"" + msg.current_name + "\" is replaced by map \"" + msg.prev_name + "\".")
                curr_map.delete()
            if not msg.save_as or (msg.save_as and curr_map):
                # rename previous map, and make edits on the previous map
                print("Map \"" + msg.prev_name + "\" is renamed to \"" + msg.current_name + "\".")
                prev_map.rename(msg.current_name)
                target_map = prev_map
            else:
                # TODO: duplicate the previous map, and set the name as current name
                # how to make a deep copy???
                print("TODO")
                return


        else:  # make edits on current map, or create one if no such map exists
            target_map = curr_map if curr_map else self.db.get_map(msg.current_name)
        
        print("Editing Map Named: " + str(target_map.get_name()))

        # handle point changes
        self.process_point_changes(target_map, msg.points)
        # handle pose changes
        self.process_pose_changes(target_map, msg.poses)  
        # handle region changes
        self.process_region_changes(target_map, msg.regions)
    

    def process_point_changes(self, target_map, point_changes):
        print("POINT BEFORE: ")
        self._print_all_points(target_map)

        for pt in point_changes:
            prev_pt = target_map.get_point(pt.prev_name)
            curr_pt = target_map.get_point(pt.current_name)
            if pt.deleted and prev_pt:
                prev_pt.delete()
            elif pt.deleted and curr_pt:
                curr_pt.delete()
            elif prev_pt:
                point_x = pt.x if pt.x != None else prev_pt.x
                point_y = pt.y if pt.y != None else prev_pt.y
                # add a new point
                target_map.add_point(pt.current_name, point_x, point_y)
                # delete the existing point
                prev_pt.delete()
            else:
                if curr_pt:
                    curr_pt.delete()
                target_map.add_point(pt.current_name, pt.x, pt.y)
        
        print("POINT AFTER: ")
        self._print_all_points(target_map)

    
    def process_pose_changes(self, target_map, pose_changes):
        print("POSE BEFORE: ")
        self._print_all_poses(target_map)

        for ps in pose_changes:
            prev_ps = target_map.get_pose(ps.prev_name)
            curr_ps = target_map.get_pose(ps.current_name)
            if ps.deleted and prev_ps:
                prev_ps.delete()
            elif ps.deleted and curr_ps:
                curr_ps.delete()
            elif prev_ps:
                pose_x = ps.x if ps.x != None else prev_ps.x
                pose_y = ps.y if ps.y != None else prev_ps.y
                pose_theta = ps.theta if ps.theta != None else prev_ps.theta
                # add a new pose
                target_map.add_pose(ps.current_name, pose_x, pose_y, pose_theta)
                # delete the existing pose
                prev_ps.delete()
            else:
                if curr_ps:
                    curr_ps.delete()
                target_map.add_pose(ps.current_name, ps.x, ps.y, ps.theta)
        
        print("POSE AFTER: ")
        self._print_all_poses(target_map)
        
        
    def process_region_changes(self, target_map, region_changes):
        print("REGION BEFORE: ")
        self._print_all_regions(target_map)

        for r in region_changes:
            prev_r = target_map.get_region(r.prev_name)
            curr_r = target_map.get_region(r.current_name)
            if r.deleted and prev_r:
                prev_r.delete()
            elif r.deleted and curr_r:
                curr_r.delete()
            else:
                updated_endpoints = []
                if prev_r:
                    if r.endpoints != None:
                        updated_endpoints = self.update_region_endpoints(prev_r.points, r.endpoints)
                    else:
                        updated_endpoints = prev_r.points
                    # add a new region
                    target_map.add_region(r.current_name, updated_endpoints)
                    # delete the existing region
                    prev_r.delete()
                else:
                    curr_r = target_map.get_region(r.current_name)
                    old_endpoints = curr_r.points if curr_r else []
                    updated_endpoints = self.update_region_endpoints(old_endpoints, r.endpoints)
                    if curr_r:
                        curr_r.delete()
                    target_map.add_region(r.current_name, updated_endpoints)        
        
        print("REGION AFTER: ")
        self._print_all_regions(target_map)


    def has_map(self, name):
        """ Return the map with the given name if it exists, return None otherwise. """
        if name != "":
            map_concept = self.db.get_concept("map")
            all_map_instances = map_concept.get_instances()
            for m in all_map_instances:
                if name == m.get_name():
                    return self.db.get_map(name)
        return None

    
    def update_region_endpoints(self, old_endpoints, new_endpoints):
        """ 
            Combine the old list of endpoints with the list of new endpoints, 
            and return a list of tuple as the updated list.
        """
        # old_endpoints (list of tuple): [p, p, p, p]
        # new_endpoints (list of Point): [_, _, p]   [_, _, p, p, p]   [p, p, p]
        old_size = len(old_endpoints)
        new_size = len(new_endpoints)
        size = max(old_size, new_size)
        updated_endpoints = []
        is_brand_new = True
        for i in range(size):
            if i < new_size and new_endpoints[i] != None:
                updated_endpoints.append((new_endpoints[i].x, new_endpoints[i].y))
            elif i >= new_size and is_brand_new:
                break
            else: 
                is_brand_new = False
                updated_endpoints.append(old_endpoints[i])

        return updated_endpoints

    
    def _print_all_points(self, target_map):
        points = target_map.get_all_points()
        for pt in points:
            print(pt.get_name())
            print("\t" + str(pt.x) + ", " + str(pt.y))
        print("--------------------------")

    
    def _print_all_poses(self, target_map):
        poses = target_map.get_all_poses()
        for ps in poses:
            print(ps.get_name())
            print("\t" + str(ps.x) + ", " + str(ps.y) + ", " + str(ps.theta))
        print("--------------------------")


    def _print_all_regions(self, target_map):
        regions = target_map.get_all_regions()
        for r in regions:
            print(r.get_name())
            for pt in r.points:
                print("\t" + str(pt))
        print("--------------------------")


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