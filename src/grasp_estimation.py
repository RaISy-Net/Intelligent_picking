import argparse
import logging

import matplotlib.pyplot as plt
import numpy as np
import torch.utils.data
from PIL import Image
import cv2
import os

from src.inference import *

from src.hardware.device import get_device
from src.inference.post_process import post_process_output
from src.utils.data.camera_data import CameraData
from src.utils.visualisation.plot import plot_results, save_results

import sys
logging.basicConfig(level=logging.INFO)

class GraspEstimation:

    def __init__(self, model):
        self.use_depth = 1
        self.use_rgb = 1 
        self.n_grasps = 1
        self.save = 0
        self.force_cpu = False

        # Load Network
        logging.info('Loading model...')
        # temp = os.getcwd()
        dir_path = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(dir_path)
        # os.chdir(dir_path)
        self.net = torch.load(model,map_location=torch.device('cpu'))
        # os.chdir(temp)
        logging.info('Done')

    def load_images(self, rgb_path, depth_path):
        # Load image
        logging.info('Loading image...')
        self.pic = Image.open(rgb_path, 'r')
        self.rgb = np.array(self.pic)
        self.pic = Image.open(depth_path, 'r')
        self.depth = np.expand_dims(np.array(self.pic), axis=2)

    def predict_grasp(self):
        # Get the compute device
        self.device = get_device(self.force_cpu)

        self.img_data = CameraData(include_depth=self.use_depth, include_rgb=self.use_rgb)

        x, depth_img, rgb_img = self.img_data.get_data(rgb=self.rgb, depth=self.depth)

        with torch.no_grad():
            xc = x.to(self.device)
            pred = self.net.predict(xc)

            q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])
            #print(pred['pos'].size())
            #print(pred['pos'])
            #print(pred['cos'])
            #print(pred['sin'])
            #print(pred['width'])
            if self.save:
                save_results(
                    rgb_img=self.img_data.get_rgb(self.rgb, False),
                    depth_img=np.squeeze(self.img_data.get_depth(self.depth)),
                    grasp_q_img=q_img,
                    grasp_angle_img=ang_img,
                    no_grasps=n_grasps,
                    grasp_width_img=width_img
                )
            else:
                fig = plt.figure(figsize=(7, 2))
                #fig.canvas.manager.window.wm_geometry("+330+440")
                gs=plot_results(fig=fig,
                            rgb_img=self.img_data.get_rgb(self.rgb, False),
                            grasp_q_img=q_img,
                            grasp_angle_img=ang_img,
                            no_grasps=self.n_grasps,
                            grasp_width_img=width_img)
                fig.savefig('./src/prediction results/img_result.pdf')
                plt.close()
                for g in gs:
                    print(g.center)
                    print(g.angle)
                    print(g.length)
                    print(g.width)
                return gs