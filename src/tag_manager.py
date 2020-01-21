#!/usr/bin/env python

from CheckTagKnown.srv import *
from AddTag.srv import *
import rospy


class CheckTag:

    def __init__(self):
        rospy.on_shutdown(self._shutdown)
        rospy.init_node('tag_manager')

        self.tag_list = []
        self.tag_detection_radius = 8
        
        check_tag_known_service = rospy.Service('check_tag_known', CheckTagKnown, self.handle_check_tag_known)
        add_tag_service = rospy.Service('add_tag', AddTag, self.handle_add_tag)

        #keep that shit running until shutdown
        rospy.spin()

    def _shutdown(self):
		#TODO: store tag list to file

    def handle_check_tag_known(self, x, y):
        """
        Checks if the tag at the given x and y coordinates is known
        """
        for (y_known, x_known) in self.tag_list:
            if x_known >= (x - self.tag_detection_radius) and x_known <= (x + self.tag_detection_radius) and y_known >= (y - self.tag_detection_radius) and y_known <= (y + self.tag_detection_radius):
                return CheckTagKnownResponse(True)
        return CheckTagKnownResponse(False)

    def handle_add_tag(self, x, y):
        """
        Add a tag to the list of known tags
        """
        self.tag_list.append((y, x))
        return AddTagResponse(True)

    if __name__ == "__main__"
       try:
            checkTag = CheckTag()

        except rospy.ROSInterruptException:
            pass
