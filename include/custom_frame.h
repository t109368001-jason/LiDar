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
                    
                    this->file_name = myFunction::millisecondToString(this->time_stamp);
                    
                    std::string png_file = output_dir + this->file_name + ".png";
                    while(myFunction::fileExists(png_file))
                    {
                        this->time_stamp += std::chrono::milliseconds(33);
                        this->file_name = myFunction::millisecondToString(this->time_stamp);
                    
                        png_file = output_dir + this->file_name + ".png";
                    }
                    stbi_write_png( png_file.c_str(), frameColor.get_width(), frameColor.get_height(), frameColor.get_bytes_per_pixel(), frameColor.get_data(), frameColor.get_stride_in_bytes());
                    
                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));

                    this->detect(png_file, bridge_file);

                    rs2::points points;
                    points = rs2::pointcloud().calculate(frameDepth);

                    this->entire_cloud = myFunction::points_to_pcl<PointT>(points);
                }
                return true;
            }
            
            bool save(std::string output_dir)
            {
                std::string file_name = output_dir + this->file_name;
                std::string txt_file = file_name + ".txt";
                std::string pcd_file = file_name + ".pcd";

                std::ofstream ofs(txt_file);
                ofs << "time_stamp=" << this->time_stamp.count() << std::endl;
                ofs << "entire_cloud=" << pcd_file << std::endl;
                pcl::io::savePCDFileBinary(pcd_file, *(this->entire_cloud));
                ofs << "objects=" << this->yolo_objects.size() << std::endl;
                for(int i = 0; i < this->yolo_objects.size(); i++)
                {
                    std::stringstream tmp;
                    tmp << file_name << "_" << i << ".pcd";
                    ofs << i << "=" << tmp.str() << std::endl;
                    pcl::io::savePCDFileBinary(tmp.str(), *(this->yolo_objects[i]->cloud));
                }

                std::vector<boost::shared_ptr<YoloObject<PointT>>> yolo_objects;
            }

            bool objectSegmentation(std::string tmp_dir, myClass::objectSegmentation<PointT> object_segmentation)
            {
                std::string txt_file = tmp_dir + this->file_name + ".txt";

                if(!myFunction::fileExists(txt_file))
                {
                    std::cerr << txt_file << " not found\n";
                    return false;
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

                    boost::shared_ptr<YoloObject<PointT>> temp(new YoloObject<PointT>);
                    temp->name = strs[0];
                    
                    object_segmentation.setBound(std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3]), std::stod(strs[4]));
                    temp->cloud = object_segmentation.division(this->entire_cloud);
                    
                    uint8_t r;
                    uint8_t g;
                    uint8_t b;

                    myFunction::createColor(i, r, g, b);

                    temp->cloud = myFunction::fillColor<PointT>(temp->cloud, r, g, b);
                    
                    //boost::shared_ptr<YoloObject<PointT>> temp(new YoloObject<PointT>(strs[0], std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3]), std::stod(strs[4])));
                    if(temp->cloud->points.size()) yolo_objects.push_back(temp);
                }
            }

            friend ostream& operator<<(ostream &out, CustomFrame &obj)
            {
                out << "time : " << obj.time_stamp.count() << " ms" << std::endl;
                out << "cloud : " << obj.entire_cloud->points.size() << " points" << std::endl;

                for(int i = 0; i < obj.yolo_objects.size(); i++)
                {
                    out << "obj" << i << " : " << obj.yolo_objects[i]->name << std::endl;
                }
                return out;
            }

        private:
            std::unordered_map<int, std::unordered_set<unsigned long long>> _framesMap;

            bool detect(std::string &png_file, std::string &bridge_file)
            {
                while(1)
                {
                    std::string line;
                    std::ifstream ifs(bridge_file);
                    
                    std::getline(ifs, line);
                    
                    ifs.close();

                    if(line == "")
                    {
                        std::ofstream ofs(bridge_file);
                        
                        ofs << png_file + '\n';
                        ofs.close();
                        break;
                    }

                    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                }
            }

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

	template<typename RandomIt, typename PointT>
	std::vector<boost::shared_ptr<CustomFrame<PointT>>> loadCustomFramesPart(int division_num, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            std::vector<boost::shared_ptr<CustomFrame<PointT>>> customFrames;
			for(auto it = beg; it != end; ++it)
			{
                boost::shared_ptr<CustomFrame<PointT>> customFrame(new CustomFrame<PointT>);
                std::ifstream ifs((*it));

                customFrame->file_name = boost::filesystem::path{(*it)}.stem().string();
                while(!ifs.eof())
                {
                    std::string tmp;
                    ifs >> tmp;

                    std::vector<std::string> strs;
                    boost::split(strs, tmp, boost::is_any_of("="));

                    if(strs[0] == "time_stamp")
                    {
                        customFrame->time_stamp = std::chrono::milliseconds(std::stoll(strs[1]));
                    }
                    else if(strs[0] == "entire_cloud")
                    {
                        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                        pcl::io::loadPCDFile(strs[1], *cloud);
                        customFrame->entire_cloud = cloud;
                    }
                    else if(strs[0] == "objects")
                    {
                        
                    }
                }
                customFrames.push_back(customFrame);
			}
			return customFrames;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, loadCustomFramesPart<RandomIt, PointT>, division_num, beg, mid);
		auto out = loadCustomFramesPart<RandomIt, PointT>(division_num, mid, end);
		auto out1 = handle.get();

		std::copy(out1.begin(), out1.end(), std::back_inserter(out));

		return out;
	}

    template<typename PointT>
	bool loadCustomFrames(std::string output_dir, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames)
	{
        std::vector<std::string> files;
        for (boost::filesystem::directory_entry & file : boost::filesystem::directory_iterator(output_dir))
        {
            if(file.path().extension().string() == ".txt")
            {
                files.push_back(file.path().string());
            }
        }
        std::sort(files.begin(), files.end());
        
        int division_num = myFunction::getDivNum<size_t, size_t>(files.size());

        customFrames = loadCustomFramesPart<decltype(files.begin()), PointT>(division_num, files.begin(), files.end());

/*
        for(int i = 0; i < files.size(); i++)
        {
            boost::shared_ptr<CustomFrame<PointT>> customFrame(new CustomFrame<PointT>);
            std::ifstream ifs(files[i]);
            while(!ifs.eof())
            {
                std::string tmp;
                ifs >> tmp;

                std::vector<std::string> strs;
                boost::split(strs, tmp, boost::is_any_of("="));
                if(strs[0] == "time_stamp")
                {
                    customFrame->time_stamp = std::chrono::milliseconds(std::stoll(strs[1]));
                }
                else if(strs[0] == "entire_cloud")
                {
		            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                    pcl::io::loadPCDFile(strs[1], *cloud);
                    customFrame->entire_cloud = cloud;
                }
                else if(strs[0] == "objects")
                {
                    
                }
            }
            customFrames.push_back(customFrame);
        }*/
    }
	template<typename PointT>
	bool getCustomFrames(std::string &bagFile, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, std::string &bridge_file, std::string &tmp_dir, int number = std::numeric_limits<int>::max())
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

        int finished = 0;

		while (true) 
		{
            playback.resume();
			auto frameset = pipe->wait_for_frames();
			playback.pause();

			if((frameset[0].get_frame_number() < frameNumber)||(finished >= number))
            {
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
                finished++;
			}
			frameNumber = frameset[0].get_frame_number();
		}
	}

}
#endif