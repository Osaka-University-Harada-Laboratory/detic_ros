import os
import copy
from typing import Optional, Tuple

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from detic_ros.msg import SegmentationInfo
import torch
import numpy as np

import sys
sys.path.insert(0, '/catkin_ws/src/detic_ros/scripts')  # noqa: E402

import detic
from detic.predictor import VisualizationDemo
from node_config import NodeConfig


class DeticWrapper:
    predictor: VisualizationDemo
    node_config: NodeConfig

    class DummyArgs:
        vocabulary: str

        def __init__(self, vocabulary, custom_vocabulary):
            assert vocabulary in \
                ['lvis', 'openimages', 'objects365', 'coco', 'custom']
            self.vocabulary = vocabulary
            self.custom_vocabulary = custom_vocabulary

    def __init__(self, node_config: NodeConfig):
        print("[Tsuru] init DeticWrapper")
        self._adhoc_hack_metadata_path()
        detectron_cfg = node_config.to_detectron_config()
        dummy_args = self.DummyArgs(
            node_config.vocabulary, node_config.custom_vocabulary)

        self.predictor = VisualizationDemo(detectron_cfg, dummy_args)
        self.node_config = node_config
        
        print("[Tsuru] generate object class table.")
        class_names = self.predictor.metadata.get("thing_classes", None)
        print("[Tsuru] num of classification object category: ", len(class_names))
        
        # save object class table as scv file
        import csv
        filename = "/catkin_ws/src/detic_ros/scripts/object_class_table.csv"
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            for item in class_names:
                writer.writerow([item])  # 各要素を1行として書き込む
        print("[Tsuru] save object class table as ", filename)
        

    @staticmethod
    def _adhoc_hack_metadata_path():
        # because original BUILDIN_CLASSIFIER is somehow posi-dep
        rospack = rospkg.RosPack()
        pack_path = rospack.get_path('detic_ros')
        path_dict = detic.predictor.BUILDIN_CLASSIFIER
        for key in path_dict.keys():
            path_dict[key] = os.path.join(pack_path, path_dict[key])

    def infer(self, msg: Image) -> Tuple[SegmentationInfo,
                                         Optional[Image],
                                         Optional[Image]]:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        if self.node_config.verbose:
            time_start = rospy.Time.now()

        predictions, visualized_output = self.predictor.run_on_image(img)
        # print("[Tsuru] predictions: ", predictions)
        instances = predictions['instances'].to(torch.device("cpu"))

        if self.node_config.verbose:
            time_elapsed = (rospy.Time.now() - time_start).to_sec()
            rospy.loginfo('elapsed time to inference {}'.format(time_elapsed))
            rospy.loginfo('detected {} classes'.format(len(instances)))

        # Create debug image
        if self.node_config.out_debug_img:
            debug_img = bridge.cv2_to_imgmsg(
                visualized_output.get_image(), encoding="rgb8")
            debug_img.header = msg.header
        else:
            debug_img = None

        # Create Image containing segmentation info
        # img_msg = Image(height=img.shape[0], width=img.shape[1], encoding='mono16')
        # img_msg.header = msg.header
        # img_msg.encoding = '16UC1'  # 16bit unsigned int = 0 to 65535. actually, object classification results in 0 to 1203.
        # img_msg.is_bigendian = 0
        # img_msg.step = img_msg.width * 1  # 2 bytes per pixel
        data = np.zeros((img.shape[0], img.shape[1])).astype(np.uint16)

        object_class_ids = instances.pred_classes.tolist()
        print("[Tsuru] object_class_ids: ", object_class_ids)

        # largest to smallest order to reduce occlusion.
        sorted_index = np.argsort(
            [-mask.sum() for mask in instances.pred_masks])
        for i in sorted_index:
            object_id = object_class_ids[i]
            mask = instances.pred_masks[i]
            # [tsuru] write object_id number on pixels according to mask.
            data[mask] = object_id
        # assert data.shape == (img_msg.height, img_msg.width)
        img_msg = bridge.cv2_to_imgmsg(data, encoding="mono16")
        img_msg.header.frame_id = msg.header.frame_id
        img_msg.header.stamp = msg.header.stamp
        img_msg.is_bigendian = 0
        # img_msg.step = img_msg.width * 2  # 2 bytes per pixel

        if self.node_config.out_debug_segimage:
            # print("[Tsuru] debug_segimage mode")
            debug_data = copy.deepcopy(data)
            human_friendly_scaling = 65536//(len(instances.pred_masks) + 1)
            debug_data = debug_data * human_friendly_scaling
            debug_img_msg = copy.deepcopy(img_msg)
            debug_img_msg.data = debug_data.flatten().astype(np.uint16).tolist()
        else:
            debug_img_msg = None

        # Create segmentation info message
        # class_names = self.predictor.metadata.get("thing_classes", None)
        seginfo = SegmentationInfo()
        seginfo.detected_classes = object_class_ids # it was originally text, but now it is index for class table.
        # confidence with 1.0 about background detection
        seginfo.scores = instances.scores.tolist()
        seginfo.header = msg.header
        seginfo.segmentation = img_msg

        if self.node_config.verbose:
            time_elapsed_total = (rospy.Time.now() - time_start).to_sec()
            rospy.loginfo(
                'total elapsed time in callback {}'.format(
                    time_elapsed_total))

            total_publication_delay = \
                (rospy.Time.now() - msg.header.stamp).to_sec()
            responsibility = \
                (time_elapsed_total / total_publication_delay) * 100.0
            rospy.loginfo(
                'total delay {} sec (this cb {} %)'.format(
                    total_publication_delay, responsibility))
        return seginfo, debug_img, debug_img_msg
