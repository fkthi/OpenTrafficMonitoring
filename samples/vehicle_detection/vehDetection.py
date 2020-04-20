"""
Mask R-CNN

Copyright (c) 2018 Matterport, Inc.
Licensed under the MIT License (see LICENSE for details)
Written by Waleed Abdulla

------------------------------------------------------------

Minor Changes applied from F. Kruber
Applied for 
IEEE Intelligent Vehicles Symposium - IV2020: 
"Vehicle Position Estimation with Aerial Imagery from Unmanned Aerial Vehicles"

USAGE: Run from Matlab File or directly via:

    # Train a new model starting from pre-trained COCO weights
    # Windows delimiters are used 
    python vehDetection.py train --dataset=C:\\Mask_RCNN\\datasets\\vehDetection --weights=coco   # Create the subfolder "C:\\Mask_RCNN\\datasets\\vehDetection\\train" and place training data there
    python vehDetection.py train --dataset=C:\\Mask_RCNN\\datasets\\vehDetection --weights=C:\Mask_RCNN\logs\mask_rcnn_car_0300_onlyA4_196img_191222.h5  # Create the subfolder "C:\\Mask_RCNN\\datasets\\vehDetection\\train" and place training data there

    # Resume training a model that you had trained earlier
    python vehDetection.py train --dataset=C:\\Mask_RCNN\\datasets\\vehDetection --weights=last
    python vehDetection.py train --dataset=C:\\Mask_RCNN\\datasets\\vehDetection --weights=mask_rcnn_car_0300_onlyA4_196img_191222.h5

    # Train a new model starting from ImageNet weights
    python vehDetection.py train --dataset=/path/to/vehDetection/dataset --weights=imagenet

    # Detection  (also Creates JPEGs with marked Shapes 
	python vehDetection.py detect --dataset=C:\\Mask_RCNN\\datasets\\vehDetection --subset=detect --weights=C:\Mask_RCNN\logs\mask_rcnn_car_0300_onlyA4_196img_191222.h5
	python vehDetection.py detect --dataset=C:\\Mask_RCNN\\datasets\\vehDetection --subset=From_Above_Footage_Of_Vehicular_Traffic_On_A_Busy_Street_Intersection_In_The_City_At_Night_[1080p]\\registered --weights=C:\Mask_RCNN\logs\mask_rcnn_car_0400_791imgs_200312.h5
    
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

    # We use a GPU with 24GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 1

    # Number of classes (including background)
    NUM_CLASSES = 1 + 1  # Background + car

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 135 # 135 no A4 ||  196 only Audi A4 labeled images (191220 data)
    
        # use small validation steps since the epoch is small
    VALIDATION_STEPS = 2 # Validation data set

    # Backbone network architecture
    # Supported values are: resnet50, resnet101
    # BACKBONE = resnet101   # the standard backbone
    # ALTERNATIVE (not used) 
    #BACKBONE = "resnet50"

    # Length of square anchor side in pixels
    RPN_ANCHOR_SCALES = (32, 64, 128, 256, 512)

    
    # Ratios of anchors at each cell (width/height)
    # A value of 1 represents a square anchor, and 0.5 is a wide anchor
    RPN_ANCHOR_RATIOS = [0.5, 1, 2]
    
        # If enabled, resizes instance masks to a smaller size to reduce
    # memory load. Recommended when using high-resolution images.
    USE_MINI_MASK = True #True #False
    MINI_MASK_SHAPE = (224, 224)  # (height, width) of the mini-mask
    
    #IMAGE_RESIZE_MODE = "square"
    #IMAGE_MIN_DIM = 1024
    #IMAGE_MAX_DIM = 1024
    IMAGE_RESIZE_MODE = "square"
    IMAGE_MIN_DIM = 1080
    IMAGE_MAX_DIM = 1920
    
    TRAIN_ROIS_PER_IMAGE = 400 #300
    MAX_GT_IMAGE = 200 # parking lot around 125 @ < 100 m

    # ROIs kept after tf.nn.top_k and before non-maximum suppression
    PRE_NMS_LIMIT = 5000 #high impact on detection time

    # Anchor stride
    # If 1 then anchors are created for each cell in the backbone feature map.
    # If 2, then anchors are created for every other cell, and so on.
    RPN_ANCHOR_STRIDE = 1

    DEVICE = "/gpu:0"  # /cpu:0 or /gpu:0

    # ROIs kept after non-maximum suppression (training and inference)
    POST_NMS_ROIS_TRAINING = 3000
    POST_NMS_ROIS_INFERENCE = 2000
    # Max number of final detections
    DETECTION_MAX_INSTANCES = 200
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
        image_ids.sort()    # sort by sucessive image names
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
#  Training
############################################################

def train(model):
    """Train the model."""
    # Training dataset.
    dataset_train = vehDetectionDataset()
    dataset_train.load_vehDetection(args.dataset, "train")
    dataset_train.prepare()

    # Validation dataset
    dataset_val = vehDetectionDataset()
    dataset_val.load_vehDetection(args.dataset, "val")
    dataset_val.prepare()

    # Image augmentation
    # http://imgaug.readthedocs.io/en/latest/source/augmenters.html
    """ change to your needs, e.g.
    augmentation = iaa.SomeOf((0, 2), [    #iaa.Sequential([
        iaa.Fliplr(0.5),
        iaa.Flipud(0.5),
        iaa.OneOf([iaa.Affine(rotate=90),
                   iaa.Affine(rotate=180),
                   iaa.Affine(rotate=270)]),
        iaa.Multiply((0.8, 1.5)),
        iaa.GaussianBlur(sigma=(0.0, 5.0))
    ])
    """

    augmentation = iaa.Sometimes(0.5, [  #iaa.Sequential([     #iaa.SomeOf((0, 2), [
        iaa.OneOf([iaa.Affine(rotate=30),
                   iaa.Affine(rotate=45),
                   iaa.Affine(rotate=75),
                   iaa.Affine(rotate=90),
                   iaa.Affine(rotate=105),
                   iaa.Affine(rotate=135),
                   iaa.Affine(rotate=150),
                   iaa.Affine(rotate=180),
                   iaa.Affine(rotate=225),
                   iaa.Affine(rotate=270),
                   iaa.Affine(rotate=300),
                   iaa.Affine(rotate=330),
                   iaa.Fliplr(0.5),
                   iaa.Flipud(0.5)])
    ])

    # *** This training schedule is an example. Update to your needs ***

    print("Training Heads")
    model.train(dataset_train, dataset_val,
                learning_rate=config.LEARNING_RATE,
                epochs=100,
                augmentation=augmentation,
                layers='heads')

    print("Training 5+")
    model.train(dataset_train, dataset_val,
                learning_rate=config.LEARNING_RATE/10,
                epochs=200,
                augmentation=augmentation,
                layers='5+')

    print("Training 4+")
    model.train(dataset_train, dataset_val,
                learning_rate=config.LEARNING_RATE /10,
                epochs=300,
                augmentation=augmentation,
                layers='4+')
    '''
    print("Training 3+")
    model.train(dataset_train, dataset_val,
                learning_rate=config.LEARNING_RATE /10,
                epochs=400,
                augmentation=augmentation,
                layers='3+')

    
    print("Training All Warm-Up")
    model.train(dataset_train, dataset_val,
                learning_rate=config.LEARNING_RATE /10,
                epochs=250,
                augmentation=augmentation,
                layers='all') 
    '''

############################################################
#  Detection
############################################################

def detect(model, dataset_dir, subset):
    """Run detection on images in the given directory."""
    print("Running on {}".format(dataset_dir))
    # Create directory
    if not os.path.exists(RESULTS_DIR):
        os.makedirs(RESULTS_DIR)
    submit_dir = "{:%Y%m%dT%H%M}".format(datetime.datetime.now())
    subsetStr = subset
    subsetStr = subsetStr.replace("\\","_")
    submit_dir = submit_dir + '_' + subsetStr
    submit_dir = os.path.join(RESULTS_DIR, submit_dir)
    #mask_dir = "mask"
    #mask_dir = os.path.join(RESULTS_DIR, submit_dir, mask_dir)
    os.makedirs(submit_dir)
    #os.makedirs(mask_dir)
    
    # Read dataset
    dataset = vehDetectionDataset()
    dataset.load_detect(dataset_dir, subset)
    dataset.prepare()
    
    '''
    # Load over images
    submission = []
    rrois = []
    images = []
    image_order = np.zeros([len(dataset.image_ids),1])
    scores = []
    trackersLoop = []
    rectsRot = []
    boxesRot = []
    boxesRotND = np.array(list, dtype=np.object)
'''

    # FK: create dtype object to export list als Cell in Matlab. First entry gets different shape, otherwise it will
    # not be a cell array and the shape can vary depending on the python shape (cell, normal array...)
    numImages = dataset.image_ids.size
    boxesRotCell = np.zeros((numImages+1,), dtype=np.object)
    boxesRotCell[0] = np.zeros((1,), dtype=np.object) #fake fill cell, will be deleted in matlab
    rectsRotCell = np.zeros((numImages+1,), dtype=np.object)
    rectsRotCell[0] = np.zeros((1,), dtype=np.object) #fake fill cell, will be deleted in matlab

    timer = np.zeros((numImages,), dtype=np.object)
    for image_id in dataset.image_ids:
        #t1 = time.time() # to print time needed for detection per image / mean over data set
        print(image_id)
        # Load image and run detection
        image = dataset.load_image(image_id)
        # Detect objects
        r = model.detect([image], verbose=0)[0]
        # Encode image to RLE. Returns a string of multiple lines
        source_id = dataset.image_info[image_id]["id"]
        
        '''
        #FK: Save variables
        rrois.append(r['rois'])
        scores.append(r['scores'])
        image_order[image_id,0] = image_id
        images.append(source_id)  #<class 'str'> 
'''

        # FK: use masks for rotated bbox
        masks = r['masks']
        rectsImg = [] #temporal variable per image
        boxImg = [] #temporal variable per image

        for iMask in range(masks.shape[2]):
            mask = masks[:, :, iMask].astype('uint8') * 255
             # uncomment the following if mask png image should be stored   """
            '''
            maskimg = masks[:,:,iMask] # [:,:,0] 3rd dimension for detected mask in image
            maskLink = str(os.path.join(mask_dir,dataset.image_info[image_id]["id"][:-4])),str(iMask),'.png'
            maskLink2 = ''.join(maskLink)
            cv2.imwrite(maskLink2, maskimg.astype('uint8') * 255)
            '''
            # minAreaRect: https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html
            #findContours: https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=minarearect
            # explanation: Python: cv2.findContours(image, mode, method[, contours[, hierarchy[, offset]]]) → contours, hierarchy
            # cv2.findContours(mask, 0, 1)  ---> mask=image, mode= 1 (only outer extreme contour (not inner ones!)), method=2 (Teh-Chin chain approximation algorithm)
            # cv2.findContours(mask, 1, 1)  ---> mask=image, mode= 2 (find all contours, no hierarchy), method=2 (Teh-Chin chain approximation algorithm)

            # find largerst contour, if more than 1 contour was found per mask (e.g. holes)
            contour, hierarchy = cv2.findContours(mask, 1, 1)
            cntIdx = 0 #default contour index (if only 1 contour)
            aa = np.array(contour)
            if aa.ndim == 1: #if only 1 contour found, then aa.ndim = 4
                bb = aa.shape
                nCnt = bb[0] # number of contours found
                areaCnt = np.ones(nCnt,)
                for xx in range(0, nCnt):
                    areaCnt[xx] = cv2.contourArea(contour[xx])
                cntIdx = np.argmax(areaCnt) #overwrite cntIdx according to the largest contour found
            cnt = contour[cntIdx]

            """ # solution to find only outer contour --> gives sometimes problems
            contour, hierarchy = cv2.findContours(mask, 0, 1)
            cnt = contour[-1]
            """
            
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            '''
            rectsImg.append(rect)
            '''
            boxImg.append(box)


        #FK: store rects and boxes
        '''
        rectsRot.append(rectsImg)
        boxesRot.append(boxImg)
        '''
        boxesRotCell[image_id + 1] = boxImg
        rectsRotCell[image_id + 1] = rectsImg

        # Save image with shape polygon around vehicles. Bbox and mask can be activated, s. below
        visualize.display_instances(
            image, r['rois'], r['masks'], r['class_ids'],
            dataset.class_names,  scores=None,# r['scores'],
            show_bbox=False, show_mask=False,
            colors=None,
            figsize=(19.20,10.80))   # can also add title="Predictions"
        plt.box(on=None) # plt.box(False)
        plt.savefig(os.path.join(submit_dir, dataset.image_info[image_id]["id"]))
        plt.close() # plt.clf()

        #print('Detection time is {}'.format(time.time() - t1))
        #timer[image_id] = time.time() - t1
        #if np.around(numImages % 10) == 0:
        #    print("Processing detection: ", image_id, " of ", numImages)
    print("Detection Process with Mask-RCNN Done. Files saved to ", submit_dir)
    print("Mean detection time in seconds: ", np.mean(timer))
    # FK: save to Matlab files
    saveMatFileName = submit_dir + "\maskrcnn_output.mat"
    saveMatFileName = submit_dir + "\_MRCNN_output_" + subsetStr +".mat"
    savemat(saveMatFileName,{"boxesRotCell": boxesRotCell})
'''
    savemat(saveMatFileName,
            {"roi": rrois, "images": images, "image_order": image_order, "scores": scores, "trackers": trackersLoop,
             "rectsRot": rectsRot, "boxesRot": boxesRot, "boxesRotCell": boxesRotCell, "rectsRotCell": rectsRotCell})
             '''

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
    else:
        print("'{}' is not recognized. "
              "Use 'train' or 'detect'".format(args.command))