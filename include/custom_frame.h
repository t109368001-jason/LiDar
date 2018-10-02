#ifndef CUSTOM_FRAME_H_
#define CUSTOM_FRAME_H_

#include "../include/function.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb_image_write.h"


namespace myFrame
{
    template<typename PointT>
    class YoloObject
    {
        public:
            std::string name;
            typename pcl::PointCloud<PointT>::Ptr cloud;
            YoloObject(const std::string name, typename pcl::PointCloud<PointT>::Ptr cloud)
            {
                this->name = name;
                this->cloud = cloud;
            }
    };

    template<typename PointT>
    class CustomFrame
    {
        public:
            std::string file_name;
            std::chrono::milliseconds time_stamp;
            typename pcl::PointCloud<PointT>::Ptr entire_cloud;
            std::vector<boost::shared_ptr<YoloObject<PointT>>> yolo_objects;

            bool set(rs2::frameset &frameset, std::chrono::milliseconds &startTime, std::string &bridge_file, std::string &tmp_dir)
            {
                auto frameDepth = frameset.get_depth_frame();
                auto frameColor = frameset.get_color_frame();
                if (frameDepth && frameColor)
                {
                    if (frames_map_get_and_set(rs2_stream::RS2_STREAM_ANY, frameDepth.get_frame_number())) {
                        return false;
                    }
                    if (frames_map_get_and_set(rs2_stream::RS2_STREAM_ANY, frameColor.get_frame_number())) {
                        return false;
                    }

                    this->time_stamp = startTime + std::chrono::milliseconds(int64_t(frameset.get_timestamp()));

                    if (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_DEPTH) { frameColor = rs2::colorizer().process(frameColor); }
                    
                    std::string cwd = std::string(getcwd(NULL, 0)) + '/';

                    std::string output_dir = cwd + tmp_dir;
                    if(!myFunction::fileExists(output_dir))
                    {
                        mkdir(output_dir.c_str(), 0777);
                    }

                    this->file_name = output_dir + myFunction::millisecondToString(this->time_stamp);
                    
                    std::string png_file = this->file_name + ".png";
                    while(myFunction::fileExists(png_file))
                    {
                        this->time_stamp += std::chrono::milliseconds(33);
                        this->file_name = output_dir + myFunction::millisecondToString(this->time_stamp);
                    
                        png_file = this->file_name + ".png";
                    }
                    stbi_write_png( png_file.c_str(), frameColor.get_width(), frameColor.get_height(), frameColor.get_bytes_per_pixel(), frameColor.get_data(), frameColor.get_stride_in_bytes());
                    
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));

                    this->detect(bridge_file);

                    rs2::points points;
                    points = rs2::pointcloud().calculate(frameDepth);

                    this->entire_cloud = myFunction::points_to_pcl<PointT>(points);
                }
                return true;
            }
            
            bool detect(std::string bridge_file)
            {
                std::string png_file = this->file_name + ".png\n";
                while(1)
                {
                    std::string line;
                    std::ifstream ifs(bridge_file);
                    
                    std::getline(ifs, line);
                    
                    ifs.close();

                    if(line == "")
                    {
                        std::ofstream ofs(bridge_file);
                        
                        ofs << png_file;
                        ofs.close();
                        break;
                    }

                    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                }
            }

            bool getTxt(std::string bridge_file)
            {
                std::string txt_file = this->file_name + ".txt";
                std::string png_file = this->file_name + ".png";

                while(1)
                {
                    if(myFunction::fileExists(txt_file))
                    {
                        std::cerr << "Yolo detection done" << std::endl;
                        break;
                    }
                    std::cerr << "Yolo detecting..." << '\r' << std::flush;
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                }

                std::ifstream ifs;
                ifs.open(txt_file);
                std::string line;
                
                std::vector<std::string> lines;
                while(!ifs.eof())
                {
                    std::getline(ifs, line);
                    
                    if(line == "") continue;

                    lines.push_back(line);
                }
                if(lines.size() == 0) return false;
                for(int i=0; i < lines.size(); i++)
                {
                    vector<string> strs;
                    boost::split(strs,lines[i],boost::is_any_of(" "));

                    //boost::shared_ptr<YoloObject<PointT>> temp(new YoloObject<PointT>(strs[0], std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3]), std::stod(strs[4])));

                    //yolo_objects.push_back(temp);
                }
                std::cerr << yolo_objects.size() << " objects" << std::endl;
            }

            friend ostream& operator<<(ostream &out, CustomFrame &obj)
            {
                out << "time : " << obj.time_stamp << std::endl;
                for(int i = 0; i < obj.yolo_objects.size(); i++)
                {
                    out << "obj" << i << " : " << obj.yolo_objects[i]->name << std::endl;
                }
            }

        private:
            std::unordered_map<int, std::unordered_set<unsigned long long>> _framesMap;

            bool frames_map_get_and_set(rs2_stream streamType, unsigned long long frameNumber)
            {
                if (_framesMap.find(streamType) == _framesMap.end()) {
                    _framesMap.emplace(streamType, std::unordered_set<unsigned long long>());
                }

                auto & set = _framesMap[streamType];
                bool result = (set.find(frameNumber) != set.end());

                if (!result) {
                    set.emplace(frameNumber);
                }

                return result;
            }
    };

	template<typename PointT>
	bool getCustomFrames(std::string bagFile, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, std::string bridge_file, std::string &tmp_dir)
	{
        rs2::config cfg;
        auto pipe = std::make_shared<rs2::pipeline>();

        cfg.enable_device_from_file(bagFile);
        std::chrono::milliseconds bagStartTime = myFunction::bagFileNameToMilliseconds(bagFile);

        rs2::pipeline_profile selection = pipe->start(cfg);

		auto device = pipe->get_active_profile().get_device();
		rs2::playback playback = device.as<rs2::playback>();
		playback.set_real_time(false);

		auto duration = playback.get_duration();
		int progress = 0;
		auto frameNumber = 0ULL;

		while (true) 
		{
            playback.resume();
			auto frameset = pipe->wait_for_frames();
			playback.pause();

			//int posP = static_cast<int>(playback.get_position() * 100. / duration.count());
			
			if (frameset[0].get_frame_number() < frameNumber) {
				break;
			}

			if(frameNumber == 0ULL)
			{
				bagStartTime -= std::chrono::milliseconds(int64_t(frameset.get_timestamp()));
			}
			
			boost::shared_ptr<CustomFrame<PointT>> customFrame(new CustomFrame<PointT>);

			if(customFrame->set(frameset, bagStartTime, bridge_file, tmp_dir))
			{
				customFrames.push_back(customFrame);
			}
			frameNumber = frameset[0].get_frame_number();
		}
	}

}
#endif