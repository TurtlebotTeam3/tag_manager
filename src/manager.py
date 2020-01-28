#!/usr/bin/env python

import os
import rospy
from tag_manager.srv import CheckTagKnown, CheckTagKnownResponse
from tag_manager.srv import AddTag, AddTagResponse
from tag_manager.srv import GetTags, GetTagsResponse

from tag_manager.msg import TagList, TagPoint

from std_msgs.msg import Bool

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg._OccupancyGrid import OccupancyGrid


class TagManager:

    def __init__(self):      
        rospy.on_shutdown(self._shutdown)
        rospy.init_node('tag_manager')

        script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
        rel_path = "../StoredTag/tags.store"
        self.abs_file_path = os.path.join(script_dir, rel_path)

        self.tag_list = []
        self.tag_detection_radius = 10
        self._load_tags()

        self.mapSub = rospy.Subscriber('/map', OccupancyGrid, self._map_callback)
        self.map_resolution = 0
        self.map_offset_x = 0
        self.map_offset_y = 0
        self.received_map = False

        self.markerArray = MarkerArray()
        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)

        self.check_tag_known_service = rospy.Service('check_tag_known', CheckTagKnown, self._handle_check_tag_known)
        self.add_tag_service = rospy.Service('add_tag', AddTag, self._handle_add_tag)
        self.get_tags_service = rospy.Service('get_tags', GetTag, self._handle_get_tags)

        rospy.spin()

    def _map_callback(self, data):
        self.map_resolution = data.info.resolution
        self.map_offset_x = data.info.origin.position.x
        self.map_offset_y = data.info.origin.position.y
        self.received_map = True

    def _shutdown(self):
        self._store_tags()

    def _handle_check_tag_known(self, data):
        """
        Checks if the tag at the given x and y coordinates is known
        """
        # extract the data
        x = data.x
        y = data.y

        # create the response
        response = Bool()
        response.data = False

        for (y_known, x_known) in self.tag_list:
            if x_known >= (x - self.tag_detection_radius) and x_known <= (x + self.tag_detection_radius) and y_known >= (y - self.tag_detection_radius) and y_known <= (y + self.tag_detection_radius):
                response.data = True
                break
        
        return CheckTagKnownResponse(response)

    def _handle_add_tag(self, data):
        """
        Add a tag to the list of known tags
        """
        # extract the data
        x = data.x
        y = data.y

        #store
        self.tag_list.append((y, x))
        
        # create the response
        response = Bool()
        response.data = True

        if self.received_map == True:            
                self._publish_point(x, y)
        
        return AddTagResponse(response)

    def _handle_get_tags(self):
        """
        Handle request for tag list
        """
        tag_list = TagList()

        for (y, x) in self.tag_list:
            tag = TagPoint()
            tag.x = x
            tag.y = y
            tag_list.tags.append(tag)

        return GetTagsResponse

    def _load_tags(self):
        """
        If tags were stored in the fiel previously add it to the list
        """
        file = None
        try:
            print("--- Loading tags ---")
            # open the file
            file = open(self.abs_file_path, "r")

            # read every line and add to the tag list
            fileRead = file.readlines()
            count = 0
            for line in fileRead:
                count = count + 1
                coords = line.strip().split(',')
                print(coords[0] + "," + coords[1])
                self.tag_list.append((int(coords[0]), int(coords[1])))

            print("--- Loaded " + str(count) + " tags ---")
        except IOError:
            print("--- No tags loaded because ./StoredTag/tags.store does not exist---")
        finally:
            if file != None:
                file.close()

    def _store_tags(self):
        file = None
        try:
            print("--- Storeing tags ---")
            # open the file
            file = open(self.abs_file_path, "w")

            count = 0
            # write all entries in tag_list to the file
            for (y, x) in self.tag_list:
                count = count + 1
                file.write(str(y) + "," + str(x) + "\n")

            print("--- Stored " + str(count) + " tags ---")
        except IOError:
            print("--- Storing tags to file failed ---")
        finally:
            file.close()

    def _publish_point(self, x, y):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = (x * self.map_resolution) + self.map_offset_x
        marker.pose.position.y = (y * self.map_resolution) + self.map_offset_y
        marker.pose.position.z = 1

        self.markerArray.markers.append(marker)

        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        self.marker_publisher.publish(self.markerArray)
        
        print "show end"
        rospy.sleep(0.01)

if __name__ == "__main__":
    try:
        tagManager = TagManager()
    except rospy.ROSInterruptException:
        pass
