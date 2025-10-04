#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from tf2_ros.transform_broadcaster import TransformBroadcaster
from world_info_msgs.msg import WorldInfo

import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from pupil_apriltags import Detector


class AprilTag(Node):
    def __init__(self) -> None:
        super().__init__('apriltag_node')
        
        self.tfb_ = TransformBroadcaster(self)
        self.ros_clock = Clock()
        self.bridge = CvBridge()
        self.create_subscription(Image,"/Spot/kinect_color", self.spot_kinect, 1)
        self.world_info_pub = self.create_publisher(WorldInfo,"/world_info_sub", 1)
        self.world_info_msg = WorldInfo()

        self.at_detector = Detector(families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)

    def spot_kinect(self,msg:Image) -> None:
        kinect_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(kinect_img, cv2.COLOR_BGR2GRAY)
        
        tags = self.at_detector.detect(gray,
            estimate_tag_pose=True,
            camera_params=[340.6133856583575, 340.6133856583575, 160., 95.],
            tag_size=0.3 * (7/10)) # although white padding should've been 8/10

        if np.size(tags) > 0:
            for tag in list(tags):
                tag_id = tag.tag_id
                pose_R = tag.pose_R
                pose_t = tag.pose_t

                self.world_info_msg.header = msg.header
                self.world_info_msg.pose.position.x = float(pose_t[2])
                self.world_info_msg.pose.position.y = float(-pose_t[0])
                self.world_info_msg.pose.position.z = float(-pose_t[1])
                r = R.from_matrix(pose_R)
                self.world_info_msg.pose.orientation.x = r.as_quat()[0]
                self.world_info_msg.pose.orientation.y = r.as_quat()[1]
                self.world_info_msg.pose.orientation.z = r.as_quat()[2]
                self.world_info_msg.pose.orientation.w = r.as_quat()[3]
                self.world_info_msg.type = "apriltag"
                self.world_info_msg.num = str(tag_id)
                self.world_info_pub.publish(self.world_info_msg)
            
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = tag.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
                cv2.line(kinect_img, ptA, ptB, (0, 255, 0), 2)
                cv2.line(kinect_img, ptB, ptC, (0, 255, 0), 2)
                cv2.line(kinect_img, ptC, ptD, (0, 255, 0), 2)
                cv2.line(kinect_img, ptD, ptA, (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
                cv2.circle(kinect_img, (cX, cY), 5, (0, 0, 255), -1)
                # draw the tag family on the image
                tagFamily = tag.tag_family.decode("utf-8")
                cv2.putText(kinect_img, tagFamily, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # show the output image
        cv2.imshow("kinect", kinect_img)
        cv2.waitKey(1)
        

def main(args = None):
    rclpy.init(args=args)
    apriltag_node = AprilTag()

    try:
        rclpy.spin(apriltag_node)
    except KeyboardInterrupt:
        apriltag_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
