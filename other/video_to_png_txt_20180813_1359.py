#!python

import os
import time
import argparse
import io
import subprocess 
import math 
import shutil

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video_file", required=True, help="video path (required)")		#arg get video path
ap.add_argument("-w", "--weights_path", required=True, help="weights path (required)")		#arg get weight path
ap.add_argument("-f", "--fps", required=True, help="fps (required)")				#arg get fps
ap.add_argument("-export_video", required=False, action='store_true', help="export video")
ap.add_argument("-remove_image", required=False, action='store_true', help="remove_image")

args = ap.parse_args()

if __name__ == "__main__":
	video_file = args.video_file		#set video path
	weights_path = args.weights_path	#set weights path
	fps = float(args.fps)			#set fps
	export_video = args.export_video
	remove_image = args.remove_image
	data_path = "./cfg/coco.data"		#set class name data
	cfg_path = "./cfg/yolov3.cfg"		#set cfg path

	video_path = ''.join(video_file.split('/')[:(len(video_file.split('/'))-1)]) + '/'
	video_name = video_file.split('.')[0].split('/')[-1]
	output_dir = video_path + video_name
	output_dir_with_video_name = output_dir + '/' + video_name
	output_txt = output_dir_with_video_name + ".txt"
	output_video = output_dir_with_video_name + ".mp4"

	try:
		if not os.path.exists(video_file):	#check video path
			raise ValueError(os.path.abspath(video_file)+" video not found")
		if not os.path.exists(weights_path):	#check weights path
			raise ValueError(os.path.abspath(weights_path)+" weights not found")
		if not os.path.exists(data_path):	#check class name data
			raise ValueError(os.path.abspath(data_path)+" data not found")
		if not os.path.exists(cfg_path):	#check cfg path
			raise ValueError(os.path.abspath(cfg_path)+" cfg not found")
		if export_video and (fps < 1.0) and (fps <> 0):
			raise ValueError("fps less then 1")

		#get video duration
		duration_temp_file = "tmp.txt"
		os.system("ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 %s > %s" %(video_file, duration_temp_file))
		duration = float(io.FileIO(duration_temp_file, "r").readline())
		os.system("sudo rm "+duration_temp_file)
		
		#if input fps is zero, then set fps to video's fps
		if fps == 0:
			#get video fps
			duration_temp_file = "tmp.txt"
			os.system("ffmpeg -i %s 2>&1 | sed -n \"s/.*, \\(.*\\) fp.*/\\1/p\" > %s " %(video_file, duration_temp_file))
			fps = float(io.FileIO(duration_temp_file, "r").readline())
			os.system("sudo rm "+duration_temp_file)

		total_frame = int(math.ceil(fps * duration))+2

		#remove old txt
		i=1
		temp = output_dir
		while os.path.exists(temp):
			temp = output_dir + ("_%d" %(i))
			i+=1
		output_dir = temp
		if not os.path.exists(output_dir):
			os.system("mkdir -p "+output_dir)
		
		#video to png
		os.system("ffmpeg -i %s -r %s %s" %(video_file, fps, output_dir_with_video_name)+"_%d.png")

		#png list txt
		temp_path = "temp.txt"

		#remove old txt
		if os.path.exists(temp_path):
			os.system("sudo rm "+temp_path)
		
		#change png index from 0 and list png path to txt
		with io.FileIO(temp_path, "w") as file:
			for i in range(1,total_frame+1,1):
				image_path = output_dir_with_video_name+ ("_%d.png" %(i))
				image_path2 = output_dir_with_video_name+ ("_%d.png" %(i-2))
				if os.path.exists(image_path):
					if i == 1:
						os.system("sudo rm "+image_path)
						continue
					os.system("sudo mv %s %s" %(image_path, image_path2))
					file.write(image_path2+'\n')

		#YOLO detect images by txt
		os.system("sudo cat %s | sudo ./darknet detector test %s %s %s -dont_show -save_labels" %(temp_path, data_path, cfg_path, weights_path))

		#remove old images
		os.system("sudo xargs rm < %s" %(temp_path))

		#remove images list txt
		os.system("sudo rm "+temp_path)
		
		#combine and remove all detect result txt, save to [videxname].txt
		with io.FileIO(output_txt, "w") as file:
			for i in range(0,total_frame,1):
				individual_txt = output_dir_with_video_name+ ("_%d.txt" %(i))
				if os.path.exists(individual_txt):
					with io.FileIO(individual_txt, "r") as file2:
		            			lines = file2.readlines()
						for j in lines:
							file.write("%s %s" %(i, j))
					os.system("sudo rm "+individual_txt)
			
		#create video from images
		if export_video:
			image_path = output_dir_with_video_name + "_%d.png.jpg"
			os.system("ffmpeg -r 1/%s -i %s -c:v mpeg4 -q:v 0 %s" %(1/fps, image_path, output_video))

		#remove images
		if remove_image:
			files = os.listdir(output_dir)
			for item in files:
				if item.endswith(".jpg"):
					os.system("sudo rm %s" %(output_dir + '/' + item))
		#set video folder permission to 777
		os.system("sudo chmod 777 %s -R" %(video_path))
		#"""
	#when ctrl +c pressed
	except KeyboardInterrupt:
		print("^C")
