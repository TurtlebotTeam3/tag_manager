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
from nav_msgs.msg import OccupancyGrid, MapMetaData


class TagManager:

    def __init__(self):      
        rospy.on_shutdown(self._shutdown)
        rospy.init_node('tag_manager')

        self.abs_file_path=rospy.get_param("~tag_file",default="/home/daniel/catkin_ws/src/tag_manager/StoredTag/tags.store")

        self.tag_list = []
        self.tag_detection_radius = 10
        self._load_tags()

        self.map_info = MapMetaData()
        self.markerArray = MarkerArray()

        # --- Publisher ---
        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)

        # --- Services ---
        self.check_tag_known_service = rospy.Service('check_tag_known', CheckTagKnown, self._handle_check_tag_known)
        self.add_tag_service = rospy.Service('add_tag', AddTag, self._handle_add_tag)
        self.get_tags_service = rospy.Service('get_tags', GetTags, self._handle_get_tags)

        self._setup()

        rospy.spin()

    def _setup(self):
        """
        Get map meta information
        """
        map = rospy.wait_for_message('map', OccupancyGrid)
        self.map_info = map.info

    def _shutdown(self):
        self._store_tags()

    def _handle_check_tag_known(self, data):
        """
        Service handler to check if the tag at the given x and y coordinates is known.
        """
        # extract the data
        x = data.x
        y = data.y

        # create the response
        response = Bool()
        response.data = self._check_tag_x_y(x,y)
        return CheckTagKnownResponse(response)

    def _check_tag_x_y(self, x, y):
        """
        Check if the given tag is kown.
        """
        known = False
        for (y_known, x_known) in self.tag_list:
            if x_known >= (x - self.tag_detection_radius) and x_known <= (x + self.tag_detection_radius) and y_known >= (y - self.tag_detection_radius) and y_known <= (y + self.tag_detection_radius):
                known = True
                break
        return known

    def _handle_add_tag(self, data):
        """
        Service handler to add a tag to the list of known tags.
        """
        # extract the data
        x = data.x
        y = data.y


        known = self._check_tag_x_y(x,y) 

        response = Bool()
        response.data = False

        if not known:
            self.tag_list.append((y, x))
            response.data = True
            rospy.loginfo("Added tag: y=" + str(y) + " x=" + str(x))       
        self._publish_point(x, y)
        
        return AddTagResponse(response)

    def _handle_get_tags(self, data):
        """
        Handle request for tag list.
        """
        tag_list = TagList()

        for (y, x) in self.tag_list:
            tag = TagPoint()
            tag.x = x
            tag.y = y
            tag_list.tags.append(tag)

        return GetTagsResponse(tag_list)

    def _load_tags(self):
        """
        Loads tags from the file if any are stored in the file.
        """
        file = None
        try:
            rospy.loginfo("--- Loading tags ---")
            # open the file
            file = open(self.abs_file_path, "r")

            # read every line and add to the tag list
            fileRead = file.readlines()
            count = 0
            for line in fileRead:
                count = count + 1
                coords = line.strip().split(',')
                rospy.loginfo(coords[0] + "," + coords[1])
                self.tag_list.append((int(coords[0]), int(coords[1])))

            rospy.loginfo("--- Loaded " + str(count) + " tags ---")
        except IOError:
            rospy.loginfo("--- No tags loaded because ./StoredTag/tags.store does not exist---")
        finally:
            if file != None:
                file.close()

    def _store_tags(self):
        """
        Store tags to the file.
        """
        file = None
        try:
            rospy.loginfo("--- Storeing tags ---")
            # open the file
            file = open(self.abs_file_path, "w")

            count = 0
            # write all entries in tag_list to the file
            for (y, x) in self.tag_list:
                count = count + 1
                file.write(str(y) + "," + str(x) + "\n")

            rospy.loginfo("--- Stored " + str(count) + " tags ---")
        except IOError:
            rospy.loginfo("--- Storing tags to file failed ---")
        finally:
            file.close()

    def _publish_point(self, x, y):
        """
        Publish tag position so they can be visualized in RViz
        """
        marker = Marker()
        marker.header.frame_id = "Turtle4711/map"
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
        marker.pose.position.x = (x * self.map_info.resolution) + self.map_info.origin.position.x
        marker.pose.position.y = (y * self.map_info.resolution) + self.map_info.origin.position.y
        marker.pose.position.z = 1

        self.markerArray.markers.append(marker)

        id = 0
        for m in self.markerArray.markers:
            m.id = id
            id += 1

        self.marker_publisher.publish(self.markerArray)

        rospy.sleep(0.01)

if __name__ == "__main__":
    try:
        tagManager = TagManager()
    except rospy.ROSInterruptException:
        pass
