#!/usr/bin/env python3
"""Minimal ROS graph contract test."""
import unittest, rospy
from std_srvs.srv import Trigger
from danger_target_manager.msg import DangerObservation, ConfirmedDanger
class RosContract(unittest.TestCase):
    def setUp(self): self.received=[]; self.sub=rospy.Subscriber("/confirmed_danger", ConfirmedDanger, self.received.append); self.pub=rospy.Publisher("/danger_observation", DangerObservation, queue_size=10); rospy.sleep(0.5)
    def test_topics_and_reset(self):
        rospy.wait_for_service("/target_manager/reset", 3.0); response=rospy.ServiceProxy("/target_manager/reset", Trigger)(); self.assertTrue(response.success)
        for index in range(5):
            msg=DangerObservation(); msg.header.stamp=rospy.Time.now()+rospy.Duration(index*.1); msg.header.frame_id="world"; msg.center.x=1; msg.center.y=2; msg.center.z=.5; msg.valid=True; msg.detector_confidence=.9; self.pub.publish(msg); rospy.sleep(.07)
        rospy.sleep(1.0); self.assertEqual(len(self.received),1); self.assertEqual(self.received[0].header.frame_id,"world")
if __name__ == "__main__": rospy.init_node("target_manager_ros_test"); import rostest; rostest.rosrun("danger_target_manager","target_manager_ros_test",RosContract)
