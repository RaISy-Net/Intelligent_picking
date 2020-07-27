import glob
import random
import os
from imageio import imwrite
import matplotlib.pyplot as plt
from utils.dataset_processing import grasp, image
from utils.data.grasp_data import GraspDatasetBase


def save_img(rgb_img,gtbbs,name):
	fig = plt.figure(figsize=(10, 10))
	plt.ion()
	plt.clf()
	ax = plt.subplot(111)
	ax.imshow(rgb_img)
	g=gtbbs[0]
	g=g.as_grasp
	g.plot(ax)
	ax.set_title('Grasp')
	ax.axis('off')
	fig.savefig(name)
	

def rotate(rgb_file,depth_file,grasp_file,rot,final_rgb,final_depth,final_grasp,zoom=1.0):
	rgb_img = image.Image.from_file(rgb_file)
	gtbbs = grasp.GraspRectangles.load_from_cornell_file(grasp_file)
	center = gtbbs.center
	depth_img = image.DepthImage.from_tiff(depth_file)
	left = max(0, min(center[1] - 112, 0))
	top = max(0, min(center[0] - 112, 0))
	gtbbs.rotate(rot, center)
	gtbbs.offset((-top, -left))
	gtbbs.zoom(zoom, (112, 112))
	depth_img.rotate(rot, center)
	depth_img.resize((224,224))
	depth_img.zoom(zoom)
	depth_img.resize((224,224))
	rgb_img.rotate(rot, center)
	rgb_img.resize((224,224))
	rgb_img.zoom(zoom)
	rgb_img.resize((224,224))
	imwrite(final_rgb,rgb_img)
	imwrite(final_depth,depth_img)
	arr = gtbbs[0].points
	file = open(final_grasp,"w")
	for i in range(4):
		file.write(str(arr[i][1])+" "+str(arr[i][0])+"\n")
	file.close()
	
	
dir='/home/ayush/Desktop/dataset_new/'
x=800
for i in range(800):	
	grasp_file=dir+'color'+str(i)+'cpos.txt'
	rgb_file=dir+'color'+str(i)+'.png'
	depth_file=dir+'depth'+str(i)+'.png'
	for j in range(4):
		rot=random.uniform(-1.5,1.5)
		#rot=0.8
		print(rot)
		final_rgb=dir+'color'+str(x)+'.png'
		final_depth=dir+'depth'+str(x)+'.png'
		final_grasp=dir+'color'+str(x)+'cpos.txt'
		rotate(rgb_file,depth_file,grasp_file,rot,final_rgb,final_depth,final_grasp)
		x=x+1
		
		
	


