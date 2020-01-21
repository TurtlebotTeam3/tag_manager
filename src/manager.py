#!/usr/bin/env python

import rospy
from tag_manager.srv import *
from std_msgs.msg import Bool


class TagManager:

    def __init__(self):      
        rospy.on_shutdown(self._shutdown)
        rospy.init_node('tag_manager')

        self.tag_list = []
        self.tag_detection_radius = 8
        self._load_tags()

        check_tag_known_service = rospy.Service(
            'check_tag_known', CheckTagKnown, self._handle_check_tag_known)
        add_tag_service = rospy.Service('add_tag', AddTag, self._handle_add_tag)

        # keep that shit running until shutdown
        rospy.spin()

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
        
        return AddTagResponse(response)

    def _load_tags(self):
        """
        If tags were stored in the fiel previously add it to the list
        """
        file = None
        try:
            print("--- Loading tags ---")
            # open the file
            file = open("./StoredTag/tags.store", "r")

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
            file.close()

    def _store_tags(self):
        file = None
        try:
            print("--- Storeing tags ---")
            # open the file
            file = open("./StoredTag/tags.store", "w")

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

if __name__ == "__main__":
    try:
        tagManager = TagManager()
    except rospy.ROSInterruptException:
        pass
