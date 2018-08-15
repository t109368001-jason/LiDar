#!python3

import os
import argparse
 
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video_path", required=True, help="video path (required)")
ap.add_argument("-w", "--weights_path", required=True,	help="weights path (required)")
ap.add_argument("-f", "--fps", required=True,	help="fps (required)")
args = vars(ap.parse_args())
 
if __name__ == "__main__":
	video_path = args["video_path"]
	weights_path = args["weights_path"]
	fps = args["fps"]
	data_path = "./cfg/coco.data"
	cfg_path = "./cfg/yolov3.cfg"
	try:
		if not os.path.exists(video_path):
			raise ValueError(os.path.abspath(video_path)+" video not found")
		if not os.path.exists(weights_path):
			raise ValueError(os.path.abspath(weights_path)+" weights not found")
		if not os.path.exists(data_path):
			raise ValueError(os.path.abspath(data_path)+" data not found")
		if not os.path.exists(cfg_path):
			raise ValueError(os.path.abspath(cfg_path)+" cfg not found")
		
		os.system("ffmpeg -i %s -r %s %s" %(video_path, fps, video_path)+"_%04d.png")
		
		for i in range(1,9999,1):
			image_path = video_path+ ("_%04d.png" %(i))
			if os.path.exists(image_path):
				os.system("sudo ./darknet detector test %s %s %s %s -dont_show -save_labels" %(data_path, cfg_path, weights_path, image_path))
				os.system("sudo rm "+image_path)
	except KeyboardInterrupt:
		print("^C")
	print()
