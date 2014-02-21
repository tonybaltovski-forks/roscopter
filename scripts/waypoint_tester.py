#!/usr/bin/env python
import rospy
import roscopter.msg

def publish_waypt():
    rospy.init_node('waypoint_tester')

    wayptListPub = rospy.Publisher('/waypoint_list', roscopter.msg.WaypointList)

    while not rospy.is_shutdown():
        wayptListMsg = roscopter.msg.WaypointList()

        # Populate waypoint list message
        for i in range (0,20):
            wayptMsg = roscopter.msg.Waypoint()

            wayptMsg.latitude = (29.662161 + i)
            wayptMsg.longitude = (-82.3777551 + i)
            wayptMsg.altitude = 100 + i;
            wayptMsg.posAcc = 10 + i;
            wayptMsg.speedTo = 20 + i;
            wayptMsg.altitude = 30 + i;
            wayptMsg.holdTime = 40 + i;
            wayptMsg.yawFrom = 50 + i;

            wayptListMsg.waypoints.append(wayptMsg);

        rospy.sleep(1.0)
        wayptListPub.publish(wayptListMsg)

        print("Waypoints Sent")
        rospy.sleep(10.0)

if __name__ == '__main__':
    try:
        publish_waypt()
    except  rospy.ROSInterruptException:
        pass
