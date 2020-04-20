"""
Mask R-CNN

Copyright (c) 2018 Matterport, Inc.
Licensed under the MIT License (see LICENSE for details)
Written by Waleed Abdulla

------------------------------------------------------------

Minor Changes applied from F. Kruber
Applied for: 
IEEE Intelligent Vehicles Symposium - IV2020: 
"Vehicle Position Estimation with Aerial Imagery from Unmanned Aerial Vehicles"

USAGE: Run from Matlab File or directly via:
	python vehDetection_evaluate.py evaluation --dataset=C:\Mask_RCNN\datasets\vehDetection --subset=evaluation --weights=C:\Mask_RCNN\logs\mask_rcnn_car_0300_onlyA4_196img_191222.h5

"""

# Set matplotlib backend
# This has to be done before other importa that might
# set it, but only if we're running in script mode
# rather than being imported.
if __name__ == '__main__':
    import matplotlib
    # Agg backend runs without a display
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

import os
import sys
import json
import datetime
import numpy as np
import skimage.draw
from imgaug import augmenters as iaa
from pdb import set_trace as bp  # bp()  -->BreakPoint
from scipy.io import savemat
import math
import cv2

# Root directory of the project
ROOT_DIR = os.path.abspath("../../")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mrcnn.config import Config
from mrcnn import model as modellib, utils
from mrcnn import visualize

# Import MISC
from numba import jit
import os.path
import matplotlib.patches as patches
from skimage import io
from sklearn.utils.linear_assignment_ import linear_assignment
import argparse

# Path to trained weights file
COCO_WEIGHTS_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")

# Directory to save logs and model checkpoints, if not provided
# through the command line argument --logs
DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")


# Results directory
# Save submission files here
RESULTS_DIR = os.path.join(ROOT_DIR, "results/")



############################################################
#  Configurations
############################################################


class vehDetectionConfig(Config):
    """Configuration for training on the vehDetection dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "car"

    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 2

    # Number of classes (including background)
    NUM_CLASSES = 1 + 1  # Background + car

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 188 # 377 without satellite images labeled images on 27nd Aug 2019
    
        # use small validation steps since the epoch is small
    VALIDATION_STEPS = 7 # 15 labeled images without Satellite on 27nd Aug 2019

    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.9

    # Length of square anchor side in pixels
    RPN_ANCHOR_SCALES = (32, 64, 128, 256, 512)

    
    # Ratios of anchors at each cell (width/height)
    # A value of 1 represents a square anchor, and 0.5 is a wide anchor
    RPN_ANCHOR_RATIOS = [0.5, 1, 2]
    
        # If enabled, resizes instance masks to a smaller size to reduce
    # memory load. Recommended when using high-resolution images.
    USE_MINI_MASK = True #False
    MINI_MASK_SHAPE = (224, 224)  # (height, width) of the mini-mask
    
    #IMAGE_RESIZE_MODE = "square"
    #IMAGE_MIN_DIM = 1024
    #IMAGE_MAX_DIM = 1024
    IMAGE_RESIZE_MODE = "square"

    IMAGE_MIN_DIM = 1080
    IMAGE_MAX_DIM = 1920
    
    TRAIN_ROIS_PER_IMAGE = 200 #300
    MAX_GT_IMAGE = 50 #parking lot around 125

    # ROIs kept after tf.nn.top_k and before non-maximum suppression
    PRE_NMS_LIMIT = 1000 # high impact on detection time

    # Anchor stride
    # If 1 then anchors are created for each cell in the backbone feature map.
    # If 2, then anchors are created for every other cell, and so on.
    RPN_ANCHOR_STRIDE = 1

    # ROIs kept after non-maximum suppression (training and inference)
    POST_NMS_ROIS_TRAINING = 3000
    POST_NMS_ROIS_INFERENCE = 300
    # Max number of final detections
    DETECTION_MAX_INSTANCES = 50 
    # Minimum probability value to accept a detected instance
    # ROIs below this threshold are skipped
    DETECTION_MIN_CONFIDENCE = 0.5

############################################################
#  Dataset
############################################################

class vehDetectionDataset(utils.Dataset):


    def load_detect(self, dataset_dir, subset):
        """Load a subset of the nuclei dataset.

        dataset_dir: Root directory of the dataset
        subset: Subset to load. The folder name is defined in the Matlab Script:

        """
        # Add classes. We have one class.
        # Naming the dataset nucleus, and the class nucleus
        self.add_class("car", 1, "car")

        # Which subset?
        dataset_dir = os.path.join(dataset_dir, subset)
        # Get image ids from directory names
        image_ids = next(os.walk(dataset_dir))[2]
        image_ids.sort()    #FK: sort by sucessive image names
        # Add images
        for image_id in image_ids:
            self.add_image(
                "car",
                image_id=image_id,
                #path=os.path.join(dataset_dir, image_id, "images/{}.png".format(image_id)))
                    path = os.path.join(dataset_dir, image_id))
                
    def load_vehDetection(self, dataset_dir, subset):
        # Add classes. We have only one class to add.
        self.add_class("car", 1, "car")

        # Train or validation dataset?
        assert subset in ["train", "val"]
        dataset_dir = os.path.join(dataset_dir, subset)

        # Load annotations
        # VGG Image Annotator (up to version 1.6) saves each image in the form:
        # { 'filename': '28503151_5b5b7ec140_b.jpg',
        #   'regions': {
        #       '0': {
        #           'region_attributes': {},
        #           'shape_attributes': {
        #               'all_points_x': [...],
        #               'all_points_y': [...],
        #               'name': 'polygon'}},
        #       ... more regions ...
        #   },
        #   'size': 100202
        # }
        # We mostly care about the x and y coordinates of each region
        # Note: In VIA 2.0, regions was changed from a dict to a list.
        annotations = json.load(open(os.path.join(dataset_dir, "via_region_data.json")))
        annotations = list(annotations.values())  # don't need the dict keys

        # The VIA tool saves images in the JSON even if they don't have any
        # annotations. Skip unannotated images.
        annotations = [a for a in annotations if a['regions']]

        # Add images
        for a in annotations:
            # Get the x, y coordinaets of points of the polygons that make up
            # the outline of each object instance. These are stores in the
            # shape_attributes (see json format above)
            # The if condition is needed to support VIA versions 1.x and 2.x.
            if type(a['regions']) is dict:
                polygons = [r['shape_attributes'] for r in a['regions'].values()]
            else:
                polygons = [r['shape_attributes'] for r in a['regions']] 

            # load_mask() needs the image size to convert polygons to masks.
            # Unfortunately, VIA doesn't include it in JSON, so we must read
            # the image. This is only managable since the dataset is tiny.
            image_path = os.path.join(dataset_dir, a['filename'])
            image = skimage.io.imread(image_path)
            height, width = image.shape[:2]

            self.add_image(
                "car",
                image_id=a['filename'],  # use file name as a unique image id
                path=image_path,
                width=width, height=height,
                polygons=polygons)

    def load_evaluation(self, dataset_dir, subset):
        # Add classes. We have only one class to add.
        self.add_class("car", 1, "car")

        dataset_dir = os.path.join(dataset_dir, subset)

        annotations = json.load(open(os.path.join(dataset_dir, "via_region_data.json")))
        annotations = list(annotations.values())  # don't need the dict keys

        # The VIA tool saves images in the JSON even if they don't have any
        # annotations. Skip unannotated images.
        annotations = [a for a in annotations if a['regions']]

        # Add images
        for a in annotations:
            # Get the x, y coordinaets of points of the polygons that make up
            # the outline of each object instance. These are stores in the
            # shape_attributes (see json format above)
            # The if condition is needed to support VIA versions 1.x and 2.x.
            if type(a['regions']) is dict:
                polygons = [r['shape_attributes'] for r in a['regions'].values()]
            else:
                polygons = [r['shape_attributes'] for r in a['regions']]

            # load_mask() needs the image size to convert polygons to masks.
            # Unfortunately, VIA doesn't include it in JSON, so we must read
            # the image. This is only managable since the dataset is tiny.
            image_path = os.path.join(dataset_dir, a['filename'])
            image = skimage.io.imread(image_path)
            height, width = image.shape[:2]

            self.add_image(
                "car",
                image_id=a['filename'],  # use file name as a unique image id
                path=image_path,
                width=width, height=height,
                polygons=polygons)

    def load_mask(self, image_id):
        """Generate instance masks for an image.
       Returns:
        masks: A bool array of shape [height, width, instance count] with
            one mask per instance.
        class_ids: a 1D array of class IDs of the instance masks.
        """
        # If not a vehDetection dataset image, delegate to parent class.
        image_info = self.image_info[image_id]
        if image_info["source"] != "car":
            return super(self.__class__, self).load_mask(image_id)

        # Convert polygons to a bitmap mask of shape
        # [height, width, instance_count]
        info = self.image_info[image_id]
        mask = np.zeros([info["height"], info["width"], len(info["polygons"])],
                        dtype=np.uint8)
        for i, p in enumerate(info["polygons"]):
            # Get indexes of pixels inside the polygon and set them to 1
            rr, cc = skimage.draw.polygon(p['all_points_y'], p['all_points_x'])
            mask[rr, cc, i] = 1

        # Return mask, and array of class IDs of each instance. Since we have
        # one class ID only, we return an array of 1s
        return mask.astype(np.bool), np.ones([mask.shape[-1]], dtype=np.int32)

    def image_reference(self, image_id):
        """Return the path of the image."""
        info = self.image_info[image_id]
        if info["source"] == "car":
            return info["path"]
        else:
            super(self.__class__, self).image_reference(image_id)


############################################################
#  Evaluation
############################################################

def evaluation(model, dataset_dir, subset):

    # Create directory
    if not os.path.exists(RESULTS_DIR):
        os.makedirs(RESULTS_DIR)
    submit_dir = "submit_{:%Y%m%dT%H%M%S}".format(datetime.datetime.now())
    submit_dir = os.path.join(RESULTS_DIR, submit_dir)
    os.makedirs(submit_dir)

    # Read dataset
    dataset_val = vehDetectionDataset()
    dataset_val.load_evaluation(dataset_dir, subset)
    dataset_val.prepare()

    numImages_eval = dataset_val.image_ids.size
    # Prepare for saving as matlab file
    images_eval = []
    mAP_all = []
    mAP_all_range = []
    precisions_all = []
    recalls_all = []
    overlaps_all = []

    for image_id in range(0,numImages_eval):
        print(image_id)
        source_id = dataset_val.image_info[image_id]["id"]
        print(source_id)
        # Load image and ground truth data
        image, image_meta, gt_class_id, gt_bbox, gt_mask = \
            modellib.load_image_gt(dataset_val, config,
                                   image_id, use_mini_mask=False)
        molded_images = np.expand_dims(modellib.mold_image(image, config), 0)
        # Run object detection
        results = model.detect([image], verbose=0)
        r = results[0]
        # Compute AP
        mAP, precisions, recalls, overlaps = \
            utils.compute_ap(gt_bbox, gt_class_id, gt_mask,
                             r["rois"], r["class_ids"], r["scores"], r['masks'])

        # Compute AP range (0.5:0.05:0.95)
        mAP_range = \
            utils.compute_ap_range(gt_bbox, gt_class_id, gt_mask,
                             r["rois"], r["class_ids"], r["scores"], r['masks'], iou_thresholds = None, verbose = 1)


        # Append results from image
        mAP_all.append(mAP)
        mAP_all_range.append(mAP_range)
        images_eval.append(source_id)
        precisions_all.append(precisions)
        recalls_all.append(recalls)
        overlaps_all.append(overlaps)

        # Save image with shape polygon around vehicles. Bbox and mask can be activated, s. below
        visualize.display_instances(
            image, r['rois'], r['masks'], r['class_ids'],
            dataset_val.class_names,  scores=None,# r['scores'],
            show_bbox=False, show_mask=False,
            colors=None,
            figsize=(19.20,10.80))   # can also add title="Predictions"
        plt.box(on=None) # plt.box(False)
        plt.savefig(os.path.join(submit_dir, dataset_val.image_info[image_id]["id"]))
        plt.close() # plt.clf()

    print("Evaluation Process with Mask-RCNN Done. Files saved to ", submit_dir)

    print("mAP: ", np.mean(mAP_all))

    # FK: save to Matlab files
    saveMatFileName = submit_dir + "\evaluation_maskrcnn_output.mat"

    savemat(saveMatFileName,
            {"mAP_all": mAP_all, "mAP_all_range": mAP_all_range, "images_eval": images_eval, "precisions_all": precisions_all, "recalls_all": recalls_all, "overlaps_all": overlaps_all})


############################################################
#  Training
############################################################

if __name__ == '__main__':
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Train Mask R-CNN to detect vehicles.')
    parser.add_argument("command",
                        metavar="<command>",
                        help="'train' or 'detect'")
    parser.add_argument('--dataset', required=False,
                        metavar="/path/to/dataset/",
                        help='Directory of the dataset')
    parser.add_argument('--weights', required=True,
                        metavar="/path/to/weights.h5",
                        help="Path to weights .h5 file or 'coco'")
    parser.add_argument('--logs', required=False,
                        default=DEFAULT_LOGS_DIR,
                        metavar="/path/to/logs/",
                        help='Logs and checkpoints directory (default=logs/)')
    parser.add_argument('--subset', required=False,
                        metavar="Dataset sub-directory",
                        help="Subset of dataset to run prediction on")
    args = parser.parse_args()

    # Validate arguments
    if args.command == "train":
        assert args.dataset, "Argument --dataset is required for training"

    print("Weights: ", args.weights)
    print("Dataset: ", args.dataset)
    print("Logs: ", args.logs)

    # Configurations
    if args.command == "train":
        config = vehDetectionConfig()
    else:
        class InferenceConfig(vehDetectionConfig):
            # Set batch size to 1 since we'll be running inference on
            # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
            GPU_COUNT = 1
            IMAGES_PER_GPU = 1
        config = InferenceConfig()
    config.display()

    # Create model
    if args.command == "train":
        model = modellib.MaskRCNN(mode="training", config=config,
                                  model_dir=args.logs)
    else:
        model = modellib.MaskRCNN(mode="inference", config=config,
                                  model_dir=args.logs)

    # Select weights file to load
    if args.weights.lower() == "coco":
        weights_path = COCO_WEIGHTS_PATH
        # Download weights file
        if not os.path.exists(weights_path):
            utils.download_trained_weights(weights_path)
    elif args.weights.lower() == "last":
        # Find last trained weights
        weights_path = model.find_last()
    elif args.weights.lower() == "imagenet":
        # Start from ImageNet trained weights
        weights_path = model.get_imagenet_weights()
    else:
        weights_path = args.weights

    # Load weights
    print("Loading weights ", weights_path)
    if args.weights.lower() == "coco":
        # Exclude the last layers because they require a matching
        # number of classes
        model.load_weights(weights_path, by_name=True, exclude=[
            "mrcnn_class_logits", "mrcnn_bbox_fc",
            "mrcnn_bbox", "mrcnn_mask"])
    else:
        model.load_weights(weights_path, by_name=True)

    # Train or evaluate
    if args.command == "train":
        train(model)
    elif args.command == "detect":
        detect(model, args.dataset, args.subset)
    elif args.command == "evaluation":
        evaluation(model, args.dataset, args.subset)
    else:
        print("'{}' is not recognized. "
              "Use 'train' or 'detect'".format(args.command))