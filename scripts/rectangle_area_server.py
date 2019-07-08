#!/usr/bin/env python3

from ros_basics_tutorials.srv import RectangleAreaService
from ros_basics_tutorials.srv import RectangleAreaServiceRequest
from ros_basics_tutorials.srv import RectangleAreaServiceResponse

import rospy

def handle_rectangle_area(req):
    '''
    service handle
    '''
    print("Returning [%s * %s = %s]"%(req.width, req.height, (req.width * req.height)))
    return RectangleAreaServiceResponse(req.width * req.height)

def rectangle_area_server():
    rospy.init_node('rectangle_area_server')
    s = rospy.Service('rectangle_area', RectangleAreaService, handle_rectangle_area)
    print("Ready to calculate rectangle_area.")
    rospy.spin()
    
if __name__ == "__main__":
    rectangle_area_server()