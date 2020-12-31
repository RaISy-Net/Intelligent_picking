'''
Team : X-Ash-A-12

To generate Depth Images from a single RGB image of any size and run grasp estimation on it

Refer to paper - High Quality Monocular Depth Estimation via Transfer Learning[arxiv.org/abs/1812.11941]
To download model used here - https://s3-eu-west-1.amazonaws.com/densedepth/nyu.h5
'''


import argparse
import glob
import logging
import os

import cv2
import matplotlib
import numpy as np
import torch.utils.data
from PIL import Image

from hardware.device import get_device
from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.visualisation.plot import plot_results, save_results

# Keras / TensorFlow
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '5'
from keras.models import load_model
from matplotlib import pyplot as plt

from layers import BilinearUpSampling2D
from utilities import display_images, load_images, predict, save_images

# Argument Parser
parser = argparse.ArgumentParser(
    description='Generate depth maps from rgb data and run grasp estimation on it'
)
parser.add_argument('--model',
                    default='nyu.h5',
                    type=str,
                    help='Trained Keras model file.')
parser.add_argument('--network',
                    type=str,
                    help='Path to saved network to evaluate',
                    default='epoch_30_iou_0.97')
parser.add_argument('--n_grasps', type=int, default=1,help='Number of grasps to consider per image')
parser.add_argument('--input',
                    default='examples/*.png',
                    type=str,
                    help='Input filename or folder.')
parser.add_argument('--save', type=int, default=0, help='Save the results')
args = parser.parse_args()


def save_depth():
    # Custom object needed for inference and training
    custom_objects = {
        'BilinearUpSampling2D': BilinearUpSampling2D,
        'depth_loss_function': None
    }
    print('Loading model...')
    # Load model into GPU / CPU
    model = load_model(args.model,
                       custom_objects=custom_objects,
                       compile=False)
    print('\nModel loaded ({0}).'.format(args.model))
    # Input images
    inputs = load_images(glob.glob(args.input))
    print('\nLoaded ({0}) images of size {1}.'.format(inputs.shape[0],
                                                      inputs.shape[1:]))
    # Compute results
    outputs = predict(model, inputs)
    save_images('result.png', outputs, is_colormap=False)


def transform():
    rgb = cv2.imread(args.input)
    depth = cv2.imread('./result.png', cv2.IMREAD_GRAYSCALE)
    rgb = cv2.resize(rgb, (224, 224), cv2.INTER_AREA)
    depth = cv2.resize(depth, (224, 224), cv2.INTER_AREA)
    cv2.imwrite('rgb.png', rgb)
    cv2.imwrite('depth.png', depth)


def grasp():
    # Load image
    logging.info('Loading image...')
    pic = Image.open('rgb.png', 'r')
    rgb = np.array(pic)
    pic = Image.open('depth.png', 'r')
    depth = np.expand_dims(np.array(pic), axis=2)

    # Load Network
    logging.info('Loading model...')
    net = torch.load(args.network, map_location=torch.device('cpu'))
    logging.info('Done')

    # Get the compute device
    device = get_device(False)

    img_data = CameraData(include_depth=1, include_rgb=1)

    x, depth_img, rgb_img = img_data.get_data(rgb=rgb, depth=depth)

    with torch.no_grad():
        xc = x.to(device)
        pred = net.predict(xc)

        q_img, ang_img, width_img = post_process_output(
            pred['pos'], pred['cos'], pred['sin'], pred['width'])

        if args.save:
            save_results(rgb_img=img_data.get_rgb(rgb, False),
                         depth_img=np.squeeze(img_data.get_depth(depth)),
                         grasp_q_img=q_img,
                         grasp_angle_img=ang_img,
                         no_grasps=args.n_grasps,
                         grasp_width_img=width_img)
        else:
            fig = plt.figure(figsize=(12, 3))
            gs = plot_results(fig=fig,
                              rgb_img=img_data.get_rgb(rgb, False),
                              grasp_q_img=q_img,
                              grasp_angle_img=ang_img,
                              no_grasps=args.n_grasps,
                              grasp_width_img=width_img)
            fig.savefig('img_result.png')


if __name__ == "__main__":
    save_depth()
    transform()
    grasp()
