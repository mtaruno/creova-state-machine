#!/usr/bin/env python
import rospy
import json
import threading
from std_msgs.msg import String
from string_service_demo.srv import StringService, StringServiceResponse

published_once = False  # 控制是否已发布

def publish_status_once(pub):
    status_msgs = ["starting", "navigating", "manipulating", "deliverying", "done"]
    for msg in status_msgs:
        pub.publish(msg)
        rospy.loginfo("Published status: %s", msg)
        rospy.sleep(10)  # 每个状态之间间隔10秒

def handle(req):
    global published_once
    rospy.loginfo("Received request: %s", req.data)
    try:
        data = json.loads(req.data)
        location_list = ["sofa", "sink", "elevator","lab", "wall"]
        if data["destination"] in location_list:
            if not published_once:
                # 异步线程来发布状态
                threading.Thread(target=publish_status_once, args=(pub,)).start()
                published_once = True
            return StringServiceResponse(result="Start")
        else:
            return StringServiceResponse(result="Invalid")
    except Exception as e:
        rospy.logerr("Invalid JSON: %s", str(e))
        return StringServiceResponse(result=req.data)

def main():
    global pub
    rospy.init_node('string_service_server')
    rospy.Service('/send_string', StringService, handle)
    rospy.loginfo("Service [/send_string] ready.")
    pub = rospy.Publisher('/robot_status', String, queue_size=10)
    rospy.loginfo("Publisher [/robot_status] ready.")
    rospy.spin()

if __name__ == '__main__':
    main()