import glob
import os
import matplotlib.pyplot as plt

from src.utils.dataset_processing import grasp, image
from src.utils.data.grasp_data import GraspDatasetBase


def save_img(rgb_img,gtbbs,name):
	fig = plt.figure(figsize=(10, 10))
	plt.ion()
	plt.clf()
	ax = plt.subplot(111)
	ax.imshow(rgb_img)
	g=gtbbs[0]
	#g=g.as_grasp
	g.plot_debug(ax)
	ax.set_title('Grasp')
	ax.axis('off')
	fig.savefig(name)

if __name__=='__main__':	
	dir='/home/ayush/Desktop/dataset/'
	x=125
	depth_img = image.DepthImage.from_tiff(dir+'depth'+str(x)+'.png')
	gtbbs = grasp.GraspRectangles.load_from_cornell_file(dir+'color'+str(x)+'cpos.txt')
	rgb_img = image.Image.from_file(dir+'color'+str(x)+'.png')
	save_img(rgb_img,gtbbs,'grasp1.png')