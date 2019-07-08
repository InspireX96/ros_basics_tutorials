#!/usr/bin/env python

import sys
import rospy
from ros_basics_tutorials.srv import RectangleAreaService
from ros_basics_tutorials.srv import RectangleAreaServiceRequest
from ros_basics_tutorials.srv import RectangleAreaServiceResponse

def rectangle_area_client(width, height):
    '''
    service request
    '''
    rospy.wait_for_service('rectangle_area')
    try:
        rectangle_area = rospy.ServiceProxy('rectangle_area', RectangleAreaService)
        resp1 = rectangle_area(width, height)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    '''
    get sys arg
    '''
    return "%s [width height]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        width = float(sys.argv[1])
        height = float(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s * %s"%(width, height))
    area = rectangle_area_client(width, height)
    print("%s * %s = %s"%(width, height, area))