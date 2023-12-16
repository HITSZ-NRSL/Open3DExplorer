#!/usr/bin/env python3
"""
Take in an image (rgb or rgb-d)
Use CNN to do semantic segmantation
Out put a cloud point with semantic color registered
\author Xuan Zhang
\date May - July 2018
"""

from __future__ import division
from __future__ import print_function

import sys

from torch._C import dtype
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'
import random

import numpy as np
# sys.path.append('/home/hitwzh/safe_exploration/src/semantic_slam/semantic_cloud/include/color_pcl_generator')
# sys.path.append('/home/hitwzh/cv_bridge/devel/lib/python3/dist-packages')
# sys.path.remove('/home/hitwzh/safe_exploration/devel/lib/python2.7/dist-packages')
# print("path", sys.path)

from sensor_msgs.msg import PointCloud2
from color_pcl_generator_newnet import ColorPclGenerator
from color_pcl_generator_newnet import PointType 
import message_filters
import time

from skimage.transform import resize
import cv2

import torch
from ptsemseg.models import get_model
from ptsemseg.utils import convert_state_dict

from torch.utils.data import DataLoader
from torch.utils.data import Dataset as BaseDataset

import segmentation_models_pytorch as smp

import albumentations as albu 
from PIL import Image as Img
from PIL import ImageTk
import matplotlib.pyplot as plt
import threading
from scipy.io import loadmat

def to_tensor(x, **kwargs):
    return x.transpose(2, 0, 1).astype('float32')

def get_preprocessing(preprocessing_fn):
    _transform = [
        albu.Lambda(image=preprocessing_fn),
        albu.Lambda(image=to_tensor, mask=to_tensor),
    ]
    return albu.Compose(_transform)

class OneShotDataset(BaseDataset):    
    def __init__(
            self, 
            input_image, # 
            classes=None, 
            augmentation=None,
            preprocessing=None,
    ):  
        self.image = input_image
        self.preprocessing = preprocessing
    
    def __getitem__(self, i):
        image = self.image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        
        h = w = 320
        image = cv2.resize(image, (w, h))    
        
        # 
        mask = np.random.randint(low=0, high=5, size=(h, w, 1)).astype('float')
           
        if self.preprocessing:
            sample = self.preprocessing(image=image, mask=mask)
            image, mask = sample['image'], sample['mask']

        return image, mask
            
    def __len__(self):
        return 1


def color_map(N=256, normalized=False):
    """
    Return Color Map in PASCAL VOC format (rgb)
    \param N (int) number of classes
    \param normalized (bool) whether colors are normalized (float 0-1)
    \return (Nx3 numpy array) a color map
    """
    def bitget(byteval, idx):
        return ((byteval & (1 << idx)) != 0)
    dtype = 'float32' if normalized else 'uint8'
    cmap = np.zeros((N, 3), dtype=dtype)
    for i in range(N):
        r = g = b = 0
        c = i
        for j in range(8):
            r = r | (bitget(c, 0) << 7-j)
            g = g | (bitget(c, 1) << 7-j)
            b = b | (bitget(c, 2) << 7-j)
            c = c >> 3
        cmap[i] = np.array([r, g, b])
    cmap = cmap/255.0 if normalized else cmap
    return cmap

color_file = os.path.abspath(os.path.join(os.getcwd(), "../Open3DExplorer/src/semantic_slam/color150_origin.mat")) 
colors = loadmat(color_file)['colors']
# colors = loadmat('/home/nrslcar/wzh/perception_ws/src/semantic_slam/color150_binary.mat')['colors']

####### CHJ add ########
def unique(ar, return_index=False, return_inverse=False, return_counts=False):
    ar = np.asanyarray(ar).flatten()

    optional_indices = return_index or return_inverse
    optional_returns = optional_indices or return_counts

    if ar.size == 0:
        if not optional_returns:
            ret = ar
        else:
            ret = (ar,)
            if return_index:
                ret += (np.empty(0, np.bool),)
            if return_inverse:
                ret += (np.empty(0, np.bool),)
            if return_counts:
                ret += (np.empty(0, np.intp),)
        return ret
    if optional_indices:
        perm = ar.argsort(kind='mergesort' if return_index else 'quicksort')
        aux = ar[perm]
    else:
        ar.sort()
        aux = ar
    flag = np.concatenate(([True], aux[1:] != aux[:-1]))

    if not optional_returns:
        ret = aux[flag]
    else:
        ret = (aux[flag],)
        if return_index:
            ret += (perm[flag],)
        if return_inverse:
            iflag = np.cumsum(flag) - 1
            inv_idx = np.empty(ar.shape, dtype=np.intp)
            inv_idx[perm] = iflag
            ret += (inv_idx,)
        if return_counts:
            idx = np.concatenate(np.nonzero(flag) + ([ar.size],))
            ret += (np.diff(idx),)
    return ret

####### CHJ add ########
def colorEncode(labelmap, colors, mode='BGR'):
    labelmap = labelmap.astype('int')
    labelmap_rgb = np.zeros((labelmap.shape[0], labelmap.shape[1], 3),
                            dtype=np.uint8)
    for label in unique(labelmap):
        if label < 0:
            continue
        labelmap_rgb += (labelmap == label)[:, :, np.newaxis] * \
            np.tile(colors[label],
                    (labelmap.shape[0], labelmap.shape[1], 1))
        # print(labelmap_rgb)

    if mode == 'BGR':
        return labelmap_rgb[:, :, ::-1]
    else:
        return labelmap_rgb

def decode_segmap(temp, n_classes, cmap):
    """
    Given an image of class predictions, produce an bgr8 image with class colors
    \param temp (2d numpy int array) input image with semantic classes (as integer)
    \param n_classes (int) number of classes
    \cmap (Nx3 numpy array) input color map
    \return (numpy array bgr8) the decoded image with class colors
    """
    r = temp.copy()
    g = temp.copy()
    b = temp.copy()
    for l in range(0, n_classes):
        r[temp == l] = cmap[l,0]
        g[temp == l] = cmap[l,1]
        b[temp == l] = cmap[l,2]
    bgr = np.zeros((temp.shape[0], temp.shape[1], 3))
    bgr[:, :, 0] = b
    bgr[:, :, 1] = g
    bgr[:, :, 2] = r
    return bgr.astype(np.uint8)

class SemanticCloud:
    """
    Class for ros node to take in a color image (bgr) and do semantic segmantation on it to produce an image with semantic class colors (chair, desk etc.)
    Then produce point cloud based on depth information
    CNN: PSPNet (https://arxiv.org/abs/1612.01105) (with resnet50) pretrained on ADE20K, fine tuned on SUNRGBD or not
    """
    def __init__(self, gen_pcl = True):
        """
        Constructor
        \param gen_pcl (bool) whether generate point cloud, if set to true the node will subscribe to depth image
        """
        self.lock = threading.RLock()

        # Get point type
        point_type = rospy.get_param('/semantic_pcl/point_type')
        if point_type == 0:
            self.point_type = PointType.COLOR
            print('Generate color point cloud.')
        elif point_type == 1:
            self.point_type = PointType.SEMANTICS_MAX
            print('Generate semantic point cloud [max fusion].')
        elif point_type == 2:
            self.point_type = PointType.SEMANTICS_BAYESIAN
            print('Generate semantic point cloud [bayesian fusion].')
        else:
            print("Invalid point type.")
            return
        # Get image size
        self.img_width, self.img_height = rospy.get_param('/camera/width'), rospy.get_param('/camera/height')
        # Set up CNN is use semantics
        if self.point_type is not PointType.COLOR:
            print('Setting up CNN model...')
            # Set device
            self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
            # Get dataset
            dataset = rospy.get_param('/semantic_pcl/dataset')
            # Setup model
            model_name ='pspnet'
            model_path = rospy.get_param('/semantic_pcl/model_path')

            # self.n_classes = 150 # Semantic class number
            # # self.model = get_model(model_name, self.n_classes, version = 'ade20k')
            # # state = torch.load(model_path)
            # # self.model.load_state_dict(convert_state_dict(state['model_state'])) # Remove 'module' from dictionary keys
            # self.cnn_input_size = (473, 473)
            # self.mean = np.array([104.00699, 116.66877, 122.67892]) # Mean value of dataset
            
            # self.model = torch.load('/home/nrslcar/wzh/perception_ws/src/semantic_slam/models/best_model_20211223_epoch112.pth')
            # self.model = self.model.module
            # self.model.eval();
            
            self.n_classes = 150 # Semantic class number
            self.model = get_model(model_name, self.n_classes, version = 'ade20k')
            start_time = time.time()
            state = torch.load(model_path)
            print('Begin setting up torch.cuda...')
            print(time.time() - start_time)
            self.model.load_state_dict(convert_state_dict(state['model_state'])) # Remove 'module' from dictionary keys
            self.cnn_input_size = (473, 473)
            self.mean = np.array([104.00699, 116.66877, 122.67892]) # Mean value of dataset

            self.model = self.model.to(self.device)
            self.model.eval()
            self.cmap = color_map(N = self.n_classes, normalized = False) # Color map for semantic classes
        # Declare array containers
        if self.point_type is PointType.SEMANTICS_BAYESIAN:
            self.semantic_colors = np.zeros((3, self.img_height, self.img_width, 3), dtype = np.uint8) # Numpy array to store 3 decoded semantic images with highest confidences
            self.confidences = np.zeros((3, self.img_height, self.img_width), dtype = np.float32) # Numpy array to store top 3 class confidences
        # Set up ROS
        print('Setting up ROS...')
        self.bridge = CvBridge() # CvBridge to transform ROS Image message to OpenCV image
        # Semantic image publisher
        self.sem_img_pub = rospy.Publisher("/semantic_pcl/semantic_image", Image, queue_size = 1)
        self.color_recieve = False
        # Set up ros image subscriber
        # Set buff_size to average msg size to avoid accumulating delay
        if gen_pcl:
            # Point cloud frame id
            frame_id = rospy.get_param('/semantic_pcl/frame_id')
            # Camera intrinsic matrix
            fx = rospy.get_param('/camera/fx')
            fy = rospy.get_param('/camera/fy')
            cx = rospy.get_param('/camera/cx')
            cy = rospy.get_param('/camera/cy')
            intrinsic = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype = np.float32)
            self.pcl_pub = rospy.Publisher("/semantic_pcl/semantic_pcl", PointCloud2, queue_size = 1)
            self.color_sub = message_filters.Subscriber(rospy.get_param('/semantic_pcl/color_image_topic'), Image, queue_size = 1, buff_size = 30*720*1280)
            self.depth_sub = message_filters.Subscriber(rospy.get_param('/semantic_pcl/depth_image_topic'), Image, queue_size = 1, buff_size = 40*720*1280) # increase buffer size to avoid delay (despite queue_size = 1)
            self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size = 1, slop = 0.3) # Take in one color image and one depth image with a limite time gap between message time stamps
            self.ts.registerCallback(self.color_depth_callback)
            self.cloud_generator = ColorPclGenerator(intrinsic, self.img_width,self.img_height, frame_id , self.point_type)
        else:
            self.image_sub = rospy.Subscriber(rospy.get_param('/semantic_pcl/color_image_topic'), Image, self.color_callback, queue_size = 1, buff_size = 30*480*640)
        print('Ready.')

    def color_depth_callback(self, color_img_ros, depth_img_ros):
        """
        Callback function to produce point cloud registered with semantic class color based on input color image and depth image
        \param color_img_ros (sensor_msgs.Image) the input color image (bgr8)
        \param depth_img_ros (sensor_msgs.Image) the input depth image (registered to the color image frame) (float32) values are in meters
        """
        print("color timestamp: ", color_img_ros.header.stamp.secs, ".", color_img_ros.header.stamp.nsecs)
        print("depth timestamp: ", depth_img_ros.header.stamp.secs, ".", depth_img_ros.header.stamp.nsecs)

        # Convert ros Image message to numpy array
        try:
            color_img = np.frombuffer(color_img_ros.data, dtype=np.uint8).reshape(color_img_ros.height, color_img_ros.width, -1)
            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            
            # hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
            # value = 80 #whatever value you want to add
            # # cv2.add(hsv[:,:,2], value, hsv[:,:,2])
            # h, s, v = cv2.split(hsv)
            # lim = 255 - value
            # v[v > lim] = 255
            # v[v <= lim] += value
            # final_hsv = cv2.merge((h, s, v))
            # # 图像拼接
            # color_img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)

            #通道分离
            B,G,R=cv2.split(color_img)
            #对图像的灰阶通道进行直方图均衡化
            EB=cv2.equalizeHist(B)
            EG=cv2.equalizeHist(G)
            ER=cv2.equalizeHist(R)
            #三通道合成彩色图片
            color_img=cv2.merge((EB,EG,ER))

            # color_img = self.bridge.imgmsg_to_cv2(color_img_ros, "bgr8")
            depth_img = np.frombuffer(depth_img_ros.data, dtype=np.uint16).reshape(depth_img_ros.height, depth_img_ros.width, -1)
            # depth_img = self.bridge.imgmsg_to_cv2(depth_img_ros, "32FC1")
        except CvBridgeError as e:
            print(e)
        # Resize depth
        if depth_img.shape[0] is not self.img_height or depth_img.shape[1] is not self.img_width:
            depth_img = resize(depth_img, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            depth_img = depth_img.astype(np.float32)
        if self.point_type is PointType.COLOR:
            cloud_ros = self.cloud_generator.generate_cloud_color(color_img, depth_img, color_img_ros.header.stamp)
        else:
            # Do semantic segmantation
            if self.point_type is PointType.SEMANTICS_MAX:
              
              semantic_color, pred_confidence = self.predict_max(color_img)
              cloud_ros = self.cloud_generator.generate_cloud_semantic_max(color_img, depth_img, semantic_color, pred_confidence, color_img_ros.header.stamp)

            elif self.point_type is PointType.SEMANTICS_BAYESIAN:
                self.predict_bayesian(color_img)
                # Produce point cloud with rgb colors, semantic colors and confidences
                cloud_ros = self.cloud_generator.generate_cloud_semantic_bayesian(color_img, depth_img, self.semantic_colors, self.confidences, color_img_ros.header.stamp)

            # Publish semantic image
            if self.sem_img_pub.get_num_connections() > 0:
                if self.point_type is PointType.SEMANTICS_MAX:
                    semantic_color_msg = self.bridge.cv2_to_imgmsg(semantic_color, encoding="bgr8")
                else:
                    semantic_color_msg = self.bridge.cv2_to_imgmsg(self.semantic_colors[0], encoding="bgr8")
                self.sem_img_pub.publish(semantic_color_msg)

        # Publish point cloud
        self.pcl_pub.publish(cloud_ros)

    def predict_max(self, img):
        """
        Do semantic prediction for max fusion
        \param img (numpy array rgb8)
        """
        class_probs = self.predict(img)
        # Take best prediction and confidence
        pred_confidence, pred_label = class_probs.max(1)
        pred_confidence = pred_confidence.squeeze(0).cpu().numpy()
        pred_label = pred_label.squeeze(0).cpu().numpy()
        pred_label = resize(pred_label, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
        pred_label = pred_label.astype(np.int)
        # Add semantic color
        # semantic_color = decode_segmap(pred_label, self.n_classes, self.cmap)
        
        semantic_color = colorEncode(pred_label, colors)

        pred_confidence = resize(pred_confidence, (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)
        return (semantic_color, pred_confidence)
    

    def predict_newnet(self, img):
      
      input_image_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
      
      # here convert rgb to bgr
      test_dataset = OneShotDataset(input_image=input_image_RGB, preprocessing=get_preprocessing(smp.encoders.get_preprocessing_fn("resnet50","imagenet")))

      image, _ = test_dataset[0]
      # 
      x_tensor = torch.from_numpy(image).to('cuda').unsqueeze(0)
      start_time = time.time()
      self.lock.acquire()
      try:
        best_model = torch.load('~/Open3DExplorer/src/semantic_slam/models/pspnet_50_ade20k.pth')
        best_model = best_model.module

        best_model.eval();
        pr_mask = best_model.predict(x_tensor)
      finally:
        # 修改完成，释放锁
        self.lock.release()

      end_time = time.time()
      print("inference time=", round(end_time-start_time, 3), "s")

      pred_confidence, pred_label = pr_mask.max(1)
      pred_confidence = pred_confidence.squeeze(0).cpu().numpy()
      pred_label = pred_label.squeeze(0).cpu().numpy()
      pred_label = resize(pred_label, (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
      pred_label = pred_label.astype(np.int)
      # Add semantic color
      pred_confidence = resize(pred_confidence, (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)
      # print(pred_confidence)
      
      # print("pr_mask_shape: ", pr_mask.shape)
      # print(type(pr_mask))
      # print('tensor:',pr_mask)

      pr_mask = pr_mask.squeeze().cpu().numpy()
      # print("pr_mask_shape: ", pr_mask.shape)
      # print(type(pr_mask))

      pr_mask = (np.argmax(pr_mask, axis=0)).astype(np.uint8)
      # print("pr_mask_shape: ", pr_mask.shape)
      # print(type(pr_mask))

      pr_mask_color = self.get_color_pallete(pr_mask)   # 这里得到的是P mode，需要转换成RGB mode
      # print(type(pr_mask_color))
      # print("mode: ", pr_mask_color.mode)
      pr_mask_color = pr_mask_color.convert('RGB')
      # print(type(pr_mask_color))
      # print("mode: ", pr_mask_color.mode)

      h, w, _ = img.shape
      # image = Img.open("/home/hitwzh/wangzhihao/test/save.png")
      # print(type(image))
      # print(image.mode)
      # image.show()
      # # img__ = cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)  
      # cv2.imshow("OpenCV",np.asarray(image))  
      # cv2.waitKey()

      pr_mask_color_resized = pr_mask_color.resize((w, h))
      # print(type(pr_mask_color_resized))
      # print(pr_mask_color_resized.mode)
      # pr_mask_color.show()
      pr_mask_color_cv2 = cv2.cvtColor(np.asarray(pr_mask_color_resized), cv2.COLOR_RGB2BGR)
      # pr_mask_color_cv2_BGR = cv2.cvtColor(pr_mask_color_cv2, cv2.COLOR_RGB2BGR)
      
      # cv2.imshow("cv2", np.asarray(pr_mask_color_resized))
      # cv2.waitKey()
      # pr_mask_color_array = np.asanyarray(pr_mask_color_resized)
      # print('semantic_color.shape: ', pr_mask_color_cv2.shape)
      # print('semantic_color.shape: ', pr_mask_color_cv2)

      # self.visualize(image = img, predicted_mask = pr_mask_color_cv2)
      return (pr_mask_color_cv2, pred_confidence)


    def get_color_pallete(self, npimg, dataset='rellis_3d'): 
        
        Rellis3D_pallete = [
            0, 0, 255,        # 0: void
            0, 0, 255,    # 1: dirt
            0, 255, 0,      # 2: grass
            0, 255, 0,      # 3: tree
            0, 0, 255,    # 4: pole
            0, 0, 255,    # 5: water
            0, 0, 255,      # 6: sky
            0, 0, 255,    # 7: vehicle
            0, 0, 255,    # 8: object
            0, 0, 255,     # 9: asphalt
            0, 0, 255,      # 10: building
            0, 0, 255,      # 11: log
            0, 0, 255,  # 12: person
            0, 0, 255,    # 13: fence
            0, 255, 0,  # 14: bush
            0, 0, 255,   # 15: concrete  gray wall
            0, 0, 255,   # 16: barrier
            0, 0, 255,  # 17: puddle
            0, 0, 255,     # 18: mud
            0, 0, 255,   # 19: rubble
        ]

        if dataset == 'rellis_3d':
          npimg[npimg == -1] = 255
          out_img = Img.fromarray(npimg.astype('uint8'))
          out_img.putpalette(Rellis3D_pallete)
          return out_img


    def visualize(self, **images):
        n = len(images)
        plt.figure(figsize=(16, 5))
        for i, (name, image) in enumerate(images.items()):
            plt.subplot(1, n, i + 1)
            plt.xticks([])
            plt.yticks([])
            plt.title(' '.join(name.split('_')).title())
            plt.imshow(image)
        plt.show()

    # def visualize(self, images):
    #     plt.figure(figsize=(16, 5))
    #     plt.imshow(images)
    #     plt.show()

    def predict_bayesian(self, img):
        """
        Do semantic prediction for bayesian fusion
        \param img (numpy array rgb8)
        """
        class_probs = self.predict(img)
        # Take 3 best predictions and their confidences (probabilities)
        pred_confidences, pred_labels  = torch.topk(input = class_probs, k = 3, dim = 1, largest = True, sorted = True)
        pred_labels = pred_labels.squeeze(0).cpu().numpy()
        pred_confidences = pred_confidences.squeeze(0).cpu().numpy()
        # Resize predicted labels and confidences to original image size
        for i in range(pred_labels.shape[0]):
            pred_labels_resized = resize(pred_labels[i], (self.img_height, self.img_width), order = 0, mode = 'reflect', anti_aliasing=False, preserve_range = True) # order = 0, nearest neighbour
            pred_labels_resized = pred_labels_resized.astype(np.int)
            # Add semantic class colors
            self.semantic_colors[i] = decode_segmap(pred_labels_resized, self.n_classes, self.cmap)
        for i in range(pred_confidences.shape[0]):
            self.confidences[i] = resize(pred_confidences[i], (self.img_height, self.img_width),  mode = 'reflect', anti_aliasing=True, preserve_range = True)

    def predict(self, img):
        """
        Do semantic segmantation
        \param img: (numpy array bgr8) The input cv image
        """
        img = img.copy() # Make a copy of image because the method will modify the image
        #orig_size = (img.shape[0], img.shape[1]) # Original image size
        # Prepare image: first resize to CNN input size then extract the mean value of SUNRGBD dataset. No normalization
        img = resize(img, self.cnn_input_size, mode = 'reflect', anti_aliasing=True, preserve_range = True) # Give float64
        img = img.astype(np.float32)
        img -= self.mean
        # Convert HWC -> CHW
        img = img.transpose(2, 0, 1)
        # Convert to tensor
        img = torch.tensor(img, dtype = torch.float32)
        img = img.unsqueeze(0) # Add batch dimension required by CNN
        with torch.no_grad():
            img = img.to(self.device)
            # Do inference
            since = time.time()
            outputs = self.model(img) #N,C,W,H
            # Apply softmax to obtain normalized probabilities
            outputs = torch.nn.functional.softmax(outputs, 1)
            return outputs


def main(args):
    rospy.init_node('semantic_cloud', anonymous=True)
    seg_cnn = SemanticCloud(gen_pcl = True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
