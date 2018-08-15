#!python3

import os
import time
import argparse
import io
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video_path", required=True, help="video path (required)")		#arg get video path
ap.add_argument("-w", "--weights_path", required=True,	help="weights path (required)")		#arg get weight path
ap.add_argument("-f", "--fps", required=True,	help="fps (required)")				#arg get fps
args = vars(ap.parse_args())
 
if __name__ == "__main__":
	video_path = args["video_path"]		#set video path
	weights_path = args["weights_path"]	#set weights path
	fps = args["fps"]			#set fps
	data_path = "./cfg/coco.data"		#set class name data
	cfg_path = "./cfg/yolov3.cfg"		#set cfg path
	try:
		if not os.path.exists(video_path):	#check video path
			raise ValueError(os.path.abspath(video_path)+" video not found")
		if not os.path.exists(weights_path):	#check weights path
			raise ValueError(os.path.abspath(weights_path)+" weights not found")
		if not os.path.exists(data_path):	#check class name data
			raise ValueError(os.path.abspath(data_path)+" data not found")
		if not os.path.exists(cfg_path):	#check cfg path
			raise ValueError(os.path.abspath(cfg_path)+" cfg not found")
		
		#video to png
		os.system("ffmpeg -i %s -r %s %s" %(video_path, fps, video_path)+"_%04d.png")
		
		#png list txt
		txt_path = video_path + ".txt"

		#remove old txt
		if os.path.exists(txt_path):
			os.system("sudo rm "+txt_path)
		
		#change png index from 0 and list png path to txt
		with io.FileIO(txt_path, "w") as file:
			for i in range(1,9999,1):
				image_path = video_path+ ("_%04d.png" %(i))
				image_path2 = video_path+ ("_%04d.png" %(i-2))
				if os.path.exists(image_path):
					if i == 1:
						os.system("sudo rm "+image_path)
						continue
					os.system("sudo mv %s %s" %(image_path, image_path2))
		            		file.write(image_path2+'\n')
		
		#YOLO detect images by txt
		os.system("sudo cat %s | sudo ./darknet detector test %s %s %s -dont_show -save_labels" %(txt_path, data_path, cfg_path, weights_path))

		#remove old images
		for i in range(0,9999,1):
			image_path = video_path+ ("_%04d.png" %(i))
			if os.path.exists(image_path):
				os.system("sudo rm "+image_path)

		#remove images list txt
		os.system("sudo rm "+txt_path)
		
		#combine and remove all detect result txt, save to [videxname].txt
		with io.FileIO(txt_path, "w") as file:
			for i in range(0,9999,1):
				output_txt_path = video_path+ ("_%04d.txt" %(i))
				if os.path.exists(output_txt_path):
					with io.FileIO(output_txt_path, "r") as file2:
		            			lines = file2.readlines()
						for j in lines:
							file.write("%s %s" %(i, j))
							print j
					os.system("sudo rm "+output_txt_path)
	#when ctrl +c pressed
	except KeyboardInterrupt:
		print("^C")
