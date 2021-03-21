#! /usr/bin/env python

import rospy

from std_srvs.srv import Trigger, TriggerResponse

def goal_reached_service_handler(req):
    res = TriggerResponse()
    res.success = True
    return res


if __name__ == '__main__':
    rospy.init_node('csci_5551_hw3_tester', anonymous=False)
    s = rospy.Service('/csci_5551/goal_reached', Trigger, goal_reached_service_handler)
    rospy.spin()
