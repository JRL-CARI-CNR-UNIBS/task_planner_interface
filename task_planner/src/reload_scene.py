#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolRequest


class Scene:
    def __init__(self):
        rospy.wait_for_service('/reset_scene')
        rospy.wait_for_service('/inbound_pick_server/remove_all_objects')
        rospy.wait_for_service('/outbound_place_server/outbound/reset_all_slot')
        rospy.wait_for_service('/inbound_pick_loader/add_objects')

        self.reset_client = rospy.ServiceProxy('/reset_scene', Trigger)
        self.reset_box_client = rospy.ServiceProxy('/inbound_pick_server/remove_all_objects', SetBool)
        self.reset_outbound_client = rospy.ServiceProxy('/outbound_place_server/outbound/reset_all_slot', SetBool)
        self.add_objs_client = rospy.ServiceProxy('/inbound_pick_loader/add_objects', SetBool)

        self.reload_scene_srv = rospy.Service('/reload_scene', Trigger, self.reload_scene)

    def reload_scene(self, request):
        rospy.loginfo("Resetting scene...")
        try:
            reset_client_response = self.reset_client()
            reset_box_client_response = self.reset_box_client(SetBoolRequest(data=True))
            reset_outbound_client_response = self.reset_outbound_client(SetBoolRequest(data=True))
            add_objs_client_response = self.add_objs_client(SetBoolRequest(data=True))
            rospy.loginfo("Scene Resetted...")

            return TriggerResponse(True, "Success")

        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)
            return TriggerResponse(False, "Fail")


def main():
    rospy.init_node('reload_scene')
    Scene()
    rospy.spin()


if __name__ == "__main__":
    main()
