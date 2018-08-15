#!python

import os
import time
import argparse
import io
import subprocess 
import math 

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video_path", required=True, help="video path (required)")		#arg get video path
ap.add_argument("-w", "--weights_path", required=True, help="weights path (required)")		#arg get weight path
ap.add_argument("-f", "--fps", required=True, help="fps (required)")				#arg get fps
ap.add_argument("-export_video", required=False, action='store_true', help="export video")
ap.add_argument("-remove_image", required=False, action='store_true', help="remove_image")

args = ap.parse_args()

if __name__ == "__main__":
	video_path = args.video_path		#set video path
	weights_path = args.weights_path	#set weights path
	fps = float(args.fps)			#set fps
	export_video = args.export_video
	remove_image = args.remove_image
	data_path = "./cfg/coco.data"		#set class name data
	cfg_path = "./cfg/yolov3.cfg"		#set cfg path
	temp = video_path.split('.')
	video_path_without_format = ''.join(temp[:(len(temp)-1)])
	txt_path = video_path_without_format + ".txt"
	temp = video_path_without_format.split('/')
	filename = ''.join(temp[(len(temp)-1):])
	try:
		if not os.path.exists(video_path):	#check video path
			raise ValueError(os.path.abspath(video_path)+" video not found")
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
		os.system("ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 %s > %s" %(video_path, duration_temp_file))
		duration = float(io.FileIO(duration_temp_file, "r").readline())
		os.system("sudo rm "+duration_temp_file)
		
		#if input fps is zero, then set fps to video's fps
		if fps == 0:
			#get video fps
			duration_temp_file = "tmp.txt"
			os.system("ffmpeg -i %s 2>&1 | sed -n \"s/.*, \\(.*\\) fp.*/\\1/p\" > %s " %(video_path, duration_temp_file))
			fps = float(io.FileIO(duration_temp_file, "r").readline())
			os.system("sudo rm "+duration_temp_file)

		total_frame = int(math.ceil(fps * duration))+2

		#video to png
		os.system("ffmpeg -i %s -r %s %s" %(video_path, fps, video_path_without_format)+"_%d.png")

		#png list txt
		temp_path = "temp.txt"

		#remove old txt
		if os.path.exists(temp_path):
			os.system("sudo rm "+temp_path)
		
		#change png index from 0 and list png path to txt
		with io.FileIO(temp_path, "w") as file:
			for i in range(1,total_frame+1,1):
				image_path = video_path_without_format+ ("_%d.png" %(i))
				image_path2 = video_path_without_format+ ("_%d.png" %(i-2))
				if os.path.exists(image_path):
					if i == 1:
						os.system("sudo rm "+image_path)
						continue
					os.system("sudo mv %s %s" %(image_path, image_path2))
		            		file.write(image_path2+'\n')
		
		#YOLO detect images by txt
		os.system("sudo cat %s | sudo ./darknet detector test %s %s %s -dont_show -save_labels" %(temp_path, data_path, cfg_path, weights_path))

		#remove old images
		for i in range(0,total_frame,1):
			image_path = video_path_without_format+ ("_%d.png" %(i))
			if os.path.exists(image_path):
				os.system("sudo rm "+image_path)
		while True:
			image_path = video_path_without_format + '/' + filename + ("_%d.jpg" %(i))
			if os.path.exists(image_path):
				os.system("sudo rm %s" %(image_path))
			else:
				break
		
		#move new images to folder
		if not os.path.exists(image_path):
			os.system("sudo mkdir "+video_path_without_format)
		for i in range(0,total_frame,1):
			image_path = video_path_without_format+ ("_%d.png.jpg" %(i))
			image_path2 = video_path_without_format + '/' + filename + ("_%d.jpg" %(i))
			if os.path.exists(image_path):
				os.system("sudo mv %s %s" %(image_path, image_path2))

		#remove images list txt
		os.system("sudo rm "+temp_path)
		
		#combine and remove all detect result txt, save to [videxname].txt
		with io.FileIO(txt_path, "w") as file:
			for i in range(0,total_frame,1):
				output_txt_path = video_path_without_format+ ("_%d.txt" %(i))
				if os.path.exists(output_txt_path):
					with io.FileIO(output_txt_path, "r") as file2:
		            			lines = file2.readlines()
						for j in lines:
							file.write("%s %s" %(i, j))
					os.system("sudo rm "+output_txt_path)
			
		#create video from images
		if export_video:
			image_path = video_path_without_format + '/' + filename + "_%d.jpg"
			new_video_path = video_path_without_format + '/' + filename + ".mp4"
			if os.path.exists(new_video_path):
				os.system("sudo rm "+new_video_path)
			os.system("ffmpeg -r 1/%s -i %s -c:v mpeg4 -q:v 0 %s" %(1/fps, image_path, new_video_path))

		#remove images
		if remove_image:
			for i in range(0,total_frame,1):
				image_path = video_path_without_format + '/' + filename + ("_%d.jpg" %(i))
				if os.path.exists(image_path):
					os.system("sudo rm %s" %(image_path))

		#set video folder permission to 777
		video_path_without_filename = video_path_without_format.split(filename)[0]
		os.system("sudo chmod 777 %s -R" %(video_path_without_filename))
		#"""
	#when ctrl +c pressed
	except KeyboardInterrupt:
		print("^C")
