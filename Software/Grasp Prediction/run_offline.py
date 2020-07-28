import argparse
import logging

import matplotlib.pyplot as plt
import numpy as np
import torch.utils.data
from PIL import Image
import cv2

from hardware.device import get_device
from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.visualisation.plot import plot_results, save_results

logging.basicConfig(level=logging.INFO)




def predict_grasp_angle(network, rgb_path, depth_path):
    #args = parse_args()

    #network = "trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch16/epoch_30_iou_0.97"
    #rgb_path = "C:/Users/yashs/OneDrive/Desktop/PS simulation/rgbd_images/color8.jpeg"
    #depth_path = "C:/Users/yashs/OneDrive/Desktop/PS simulation/rgbd_images/depth8.jpeg"
    use_depth = 1
    use_rgb = 1 
    n_grasps = 1
    save = 0
    force_cpu = False

    # Load image
    logging.info('Loading image...')
    pic = Image.open(rgb_path, 'r')
    rgb = np.array(pic)
    pic = Image.open(depth_path, 'r')
    depth = np.expand_dims(np.array(pic), axis=2)

    # Load Network
    logging.info('Loading model...')
    net = torch.load(network,map_location=torch.device('cpu'))
    logging.info('Done')

    # Get the compute device
    device = get_device(force_cpu)

    img_data = CameraData(include_depth=use_depth, include_rgb=use_rgb)

    x, depth_img, rgb_img = img_data.get_data(rgb=rgb, depth=depth)

    with torch.no_grad():
        xc = x.to(device)
        pred = net.predict(xc)

        q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])
        #print(pred['pos'].size())
        #print(pred['pos'])
        #print(pred['cos'])
        #print(pred['sin'])
        #print(pred['width'])
        if save:
            save_results(
                rgb_img=img_data.get_rgb(rgb, False),
                depth_img=np.squeeze(img_data.get_depth(depth)),
                grasp_q_img=q_img,
                grasp_angle_img=ang_img,
                no_grasps=n_grasps,
                grasp_width_img=width_img
            )
        else:
            fig = plt.figure(figsize=(10, 10))
            gs=plot_results(fig=fig,
                         rgb_img=img_data.get_rgb(rgb, False),
                         grasp_q_img=q_img,
                         grasp_angle_img=ang_img,
                         no_grasps=n_grasps,
                         grasp_width_img=width_img)
            #fig.savefig('img_result.pdf')
            for g in gs:
            	print(g.center)
            	print(g.angle)
            	print(g.length)
            	print(g.width)
            

    return gs

#predict_grasp_angle("C:/Users/yashs/Downloads/epoch_00_iou_0.01", "C:/Users/yashs/OneDrive/Desktop/Intelligent_picking-master/Software/ps simulation/Robot/simulation_images/color10.png", "C:/Users/yashs/OneDrive/Desktop/Intelligent_picking-master/Software/ps simulation/Robot/simulation_images/depth10.png")