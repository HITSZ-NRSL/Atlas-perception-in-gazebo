#!/usr/bin/python3
import sys
import os
import functools
import time

import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont, ImageOps

import rospy
import sensor_msgs.msg
from ros_rs_msgs.msg import DetectionMessage, DetectionMessages
from cv_bridge import CvBridge, CvBridgeError

path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join("../../"))

currentPath = os.path.join(path, "..")
OUTPUT_DIR = os.path.join(currentPath, 'outputs')
MODEL_PATH = os.path.join(currentPath, "model/yolox_tiny_bottle_simple.om")

import acl
from atlas_utils.acl_resource import AclResource
from atlas_utils.acl_model import Model
from atlas_utils.acl_image import AclImage
from atlas_utils.acl_dvpp import Dvpp
import atlas_utils.constants as const
import atlas_utils.utils as utils

IMAGE_SIZE = (416, 416)
CONF_TH = 0.3
NMS_TH = 0.45
CLASSES = 80
STRIDES = (8, 16, 32)
IMG_STD_MEAN = ((123.485, 116.28, 103.53), (58.395, 57.12, 57.375))
clsnames = ['bottle']
'''
CLSNAMES = [ 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
             'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
             'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
             'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
             'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
             'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
             'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
             'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
             'hair drier', 'toothbrush']
'''

def display_process_time(func):
    @functools.wraps(func)
    def decorated(*args, **kwargs):
        s1 = time.time()
        res = func(*args, **kwargs)
        s2 = time.time()
        print('%s process time %f ms' % (func.__name__, 1000*(s2-s1)))
        return res

    return decorated


def plot_one_box(x, img, color=None, label=None):
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, 2)
    # cv2.rectangle(img, (int(x[0]), int(x[1]) - 15), (int(x[0]) + 100, int(x[1]) + 2), (255, 128, 128), -1)
    cv2.putText(img, label, (int(x[0]), int(x[1]) - 8), cv2.FONT_ITALIC, 0.8, (0, 255, 0), thickness=2,
                lineType=cv2.LINE_AA)


def draw_dets(img, dets, dst, ratios=None):
    if dets is None:
        print('Nothing detect')
        return
    for x1, y1, x2, y2, conf, cls in dets:
        label = CLSNAMES[int(cls)]
        plot_one_box(x=[x1, y1, x2, y2], img=img, label=label, color=[0, 0, 255])
    if ratios is not None:
        if ratios[0] > 1:
            w = int(img.shape[0] / ratios[0])
            img = img[:, :w, :]
        if ratios[0] < 1:
            h = int(img.shape[1] * ratios[0])
            img = img[:h, :, :]
    cv2.imencode('.jpg', img)[1].tofile(dst)


class YOLOX(object):
    def __init__(self, model_path=MODEL_PATH, image_size=IMAGE_SIZE,
                 conf_thres=CONF_TH, nms_thres=NMS_TH, strides=STRIDES, std_mean=IMG_STD_MEAN):
        self.model_path = model_path
        self.input_size = image_size
        self.conf_thres = conf_thres
        self.nms_thres = nms_thres
        self.strides = strides

        self.std = np.array(std_mean[0]).reshape(1, 1, -1)
        self.mean = np.array(std_mean[1]).reshape(1, 1, -1)

        self.model = None
        self.grids = []
        self.expanded_strides = []
        self._init()

    def _load_model(self):
        print(self.model_path)
        self.model = Model(self.model_path)
        return const.SUCCESS

    def _load_dvpp(self):
        self.dvpp = Dvpp()
        return const.SUCCESS

    def _init(self):
        self._load_model()
        self._load_dvpp()
        self._make_grids()
        self.grids = np.concatenate(self.grids, axis=-2)
        self.expanded_strides = np.concatenate(self.expanded_strides, axis=-2)
        return const.SUCCESS

    def _make_grids(self):
        for stride in self.strides:
            (x_step, y_step) = (self.input_size[1]//stride, self.input_size[0]//stride)
            x, y = np.arange(x_step), np.arange(y_step)
            xv, yv = np.meshgrid(*[x, y])
            grid = np.stack((xv, yv), axis=2).reshape(1, -1, 2)
            self.grids.append(grid)
            self.expanded_strides.append(np.full((*grid.shape[:2], 1), stride))

    def _xywh2xyxy(self, x):
        y = np.zeros_like(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
        y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
        y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
        y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
        return y

    def _nms(self, boxes, scores, nms_thr):
        """Single class NMS implemented in Numpy."""
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            ovr = inter / (areas[i] + areas[order[1:]] - inter)

            inds = np.where(ovr <= nms_thr)[0]
            order = order[inds + 1]

        return keep

    def _multiclass_nms(self, boxes, scores, nms_thr, score_thr):
        """Multiclass NMS implemented in Numpy"""
        final_dets = []
        num_classes = scores.shape[-1]
        for cls_ind in range(num_classes):
            cls_scores = scores[:, cls_ind]
            valid_score_mask = cls_scores > score_thr
            if valid_score_mask.sum() == 0:
                continue
            else:
                valid_scores = cls_scores[valid_score_mask]
                valid_boxes = boxes[valid_score_mask]
                keep = self._nms(valid_boxes, valid_scores, nms_thr)
                if len(keep) > 0:
                    cls_inds = np.ones((len(keep), 1)) * cls_ind
                    dets = np.concatenate([valid_boxes[keep], valid_scores[keep, None], cls_inds], 1)
                    final_dets.append(dets)
        if len(final_dets) == 0:
            return None
        return np.concatenate(final_dets, 0)

    @display_process_time
    def pre_process(self, image):
        image_padded = np.ones([self.input_size[0], self.input_size[1], 3], dtype=np.float32) * 114.0
        r = min(self.input_size[0]/image.shape[0], self.input_size[1]/image.shape[1])
        image_resized = cv2.resize(image, (int(image.shape[1] * r), int(image.shape[0] * r)), cv2.INTER_LINEAR)
        image_padded[:int(image.shape[0] * r), :int(image.shape[1] * r), :] = image_resized

        #img = image_padded[:, :, ::-1]
        #img = (img - self.mean)/self.std
        img = image_padded
        img = np.concatenate([img[::2, ::2, :], img[1::2, ::2, :],
                              img[::2, 1::2, :], img[1::2, 1::2, :]], axis=-1)
        img = np.expand_dims(img.transpose((2, 0, 1)), axis=0)
        inp = np.ascontiguousarray(img, dtype=np.float32)
        # inp = np.fromfile('../data/inp_save.bin', dtype=np.float32).reshape(1,-1,320,320)
        return inp, r, image_padded.astype(np.uint8)

    @display_process_time
    def inferece(self, inp):
        acl.rt.set_device(0)
        outs = self.model.execute([inp, ])
        if len(outs) == 1:
            prediction = outs[0].squeeze()
        elif len(outs) > 1:
            prediction = np.concatenate(outs, axis=-2).squeeze()
        else:
            prediction = None
        return prediction

    @display_process_time
    def post_process(self, prediction):
        prediction[..., :2] = (prediction[..., :2] + self.grids) * self.expanded_strides
        prediction[..., 2:4] = np.exp(prediction[..., 2:4]) * self.expanded_strides

        bboxes = prediction[..., :4]
        scores = prediction[..., 4:5] * prediction[..., 5:]
        bboxes = self._xywh2xyxy(bboxes)
        dets = self._multiclass_nms(bboxes, scores, self.nms_thres, self.conf_thres)
        return dets

    def detect(self, image):
        inp, r, img = self.pre_process(image)
        prediction = self.inferece(inp)
        dets = self.post_process(prediction)

        return dets, r, img


def main():
    acl_resource = AclResource()
    acl_resource.init()
    """
    main
    """
    image_dir = os.path.join(currentPath, "data")
    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)
    images_list = [os.path.join(image_dir, img)
                   for img in os.listdir(image_dir)
                   if os.path.splitext(img)[1] in const.IMG_EXT]

    detector = YOLOX(model_path=MODEL_PATH, image_size=IMAGE_SIZE,
                     conf_thres=CONF_TH, nms_thres=NMS_TH,
                     strides=STRIDES, std_mean=IMG_STD_MEAN)

    for image_file in images_list:
        # Read image
        print('=== ' + os.path.basename(image_file) + '===')
        output_path = os.path.join(OUTPUT_DIR, os.path.basename(image_file))
        image = cv2.imread(image_file)
        dets, r, img = detector.detect(image)
        draw_dets(img, dets, output_path)


class object_detect_node(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber('/iris_0/realsense_camera/color/image_raw', sensor_msgs.msg.Image, self.image_receiver)
        self.result_pub = rospy.Publisher('/untracked_info', DetectionMessages, queue_size=1)

        self.cv_bridge = CvBridge()
        self.detectionresult = DetectionMessages()

        self.detector = YOLOX(model_path='yolox_tiny_bottle_simple.om', image_size=IMAGE_SIZE,
                     conf_thres=CONF_TH, nms_thres=NMS_TH,
                     strides=STRIDES, std_mean=IMG_STD_MEAN)

    def dets_clear(self):
        # Clear the detection results before every detection.
        self.detectionresult.data.clear()
        self.detectionresult.detection_num = 0
        self.detectionresult.header.stamp = None

    def image_receiver(self, img_msg):
        try:
            print('image receiving')
            self.dets_clear()
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
            np_arr = np.fromstring(img_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # print(image_np.shape)
            self.detectionresult.header.stamp = img_msg.header.stamp
            self.object_detect(image_np)
            self.result_pub.publish(self.detectionresult)
        except CvBridgeError as e:
            print(e)

    def object_detect(self, image):
        dets, _, _ = self.detector.detect(image)
        if dets is None:
            print('Nothing detected')
            #cv2.imshow('Pedestrian Detection from End Effector', image)
            #cv2.waitKey(1)
        else:
            for i in range(dets.shape[0]):
                det = DetectionMessage()
                det.x1 = int(dets[i, 0])
                det.y1 = int(dets[i, 1])
                det.x2 = int(dets[i, 2])
                det.y2 = int(dets[i, 3])
                det.score = dets[i, 4]
                det.class_pred = int(dets[i, 5])
                print("detected: class={}, x1y1x2y2={},{},{},{}, conf={}".format(clsnames[det.class_pred],
                                                                                 det.x1, det.y1, det.x2, det.y2, det.score))
                cv2.rectangle(image, (det.x1, det.y1), (det.x2, det.y2), (255, 255, 0), 2)
                cv2.putText(image, clsnames[det.class_pred], (det.x1, det.y1 - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
                #cv2.imshow('Pedestrian Detection from End Effector', image)
                #cv2.waitKey(1)

                self.detectionresult.data.append(det)
            self.detectionresult.detection_num = dets.shape[0]

    def shutdown_message(self):
        rospy.loginfo("Shutdown.")


if __name__ == '__main__':
    rospy.init_node('yolox_opcv_node_endeffect', anonymous=False)
    acl_resource = AclResource()
    acl_resource.init()
    acl.rt.set_device(0)
    detect_node = object_detect_node()
    rospy.on_shutdown(detect_node.shutdown_message)
    rospy.spin()
