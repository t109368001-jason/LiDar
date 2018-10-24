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
            typename pcl::PointCloud<PointT>::Ptr cloud2;
            int num;

            YoloObject()
            {
                this->num = 1;
            }
            void swap()
            {
		        typename pcl::PointCloud<PointT>::Ptr temp{this->cloud};
                this->cloud.reset(this->cloud2);
                this->cloud2.reset(temp);
                this->num = (this->num%2) + 1;
            }
            void swap(const int &i)
            {
                if(((i == 1)||(i == 2))&&(this->num != i))
                {
                    this->swap();
                }
            }
            void sync()
            {
                this->cloud2.reset(this->cloud);
            }
    };

#pragma region CustomFrame

    template<typename PointT>
    class CustomFrame
    {
        public:
            std::string file_name;
            std::chrono::milliseconds time_stamp;
            typename pcl::PointCloud<PointT>::Ptr full_cloud;
            std::vector<boost::shared_ptr<YoloObject<PointT>>> yolo_objects;

            bool full_cloud_isSet;
            bool objects_cloud_isSet;

            bool set(rs2::frameset &frameset, const std::chrono::milliseconds &startTime, const std::string &darknet_txt_path, const std::string &tmp_dir, const bool &postProcessing)
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
                    
                    this->file_name = myFunction::millisecondToString(this->time_stamp);
                    
                    std::string bmp_file = tmp_dir + this->file_name + ".bmp";
                    while(myFunction::fileExists(bmp_file))
                    {
                        this->time_stamp += std::chrono::milliseconds(33);
                        this->file_name = myFunction::millisecondToString(this->time_stamp);
                    
                        bmp_file = tmp_dir + this->file_name + ".bmp";
                    }
                    stbi_write_bmp( bmp_file.c_str(), frameColor.get_width(), frameColor.get_height(), frameColor.get_bytes_per_pixel(), frameColor.get_data());//, frameColor.get_stride_in_bytes());
                    
                    this->detect(tmp_dir, darknet_txt_path);

                    if(postProcessing)
                    {
                        rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
                        rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
                        rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise
                        const std::string disparity_filter_name = "Disparity";
                        rs2::disparity_transform depth_to_disparity(true);
                        rs2::disparity_transform disparity_to_depth(false);

                        frameDepth = dec_filter.process(frameDepth);
                        frameDepth = depth_to_disparity.process(frameDepth);
                        frameDepth = spat_filter.process(frameDepth);
                        frameDepth = temp_filter.process(frameDepth);
                        frameDepth = disparity_to_depth.process(frameDepth);
                    }

                    rs2::points points;
                    points = rs2::pointcloud().calculate(frameDepth);

                    this->full_cloud = myFunction::points_to_pcl<PointT>(points);
                    full_cloud_isSet = true;
                }
                return true;
            }
            
            bool detect(const std::string &tmp_dir, const std::string &darknet_txt_path)
            {
                std::string cwd = std::string(getcwd(NULL, 0)) + '/';

                std::string bmp_file = cwd + tmp_dir + this->file_name + ".bmp";
                while(1)
                {
                    std::string line;
                    std::ifstream ifs(darknet_txt_path);
                    
                    std::getline(ifs, line);
                    
                    ifs.close();

                    if(line == "")
                    {
                        std::ofstream ofs(darknet_txt_path);
                        
                        ofs << bmp_file + '\n';
                        ofs.close();
                        break;
                    }

                    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                }
            }

            bool save(const std::string &data_dir, const bool &compress, const bool &skip_full_cloud, const bool &skip_objects_cloud)
            {
                std::string file_name = data_dir + this->file_name;
                std::string txt_file = data_dir + this->file_name + ".txt";
                std::string pcd_file = this->file_name + ".pcd";
                
                std::ofstream ofs(txt_file);
                ofs << "time_stamp=" << this->time_stamp.count() << std::endl;
                if(compress)
                {
                    ofs << "full_cloud=" << pcd_file << std::endl;
                    if((full_cloud_isSet)&&(!skip_full_cloud))
                    {
                        pcl::io::savePCDFileBinaryCompressed(data_dir + pcd_file, *(this->full_cloud));
                    }
                    if((objects_cloud_isSet)&&(!skip_objects_cloud))
                    {
                        for(int i = 0; i < this->yolo_objects.size(); i++)
                        {
                            if(this->yolo_objects[i]->cloud->points.size() > 0)
                            {
                                std::stringstream tmp;
                                tmp << this->file_name << "_" << i << ".pcd";
                                ofs << i << '_' << this->yolo_objects[i]->name << "=" << tmp.str() << std::endl;

                                pcl::io::savePCDFileBinaryCompressed(data_dir + tmp.str(), *(this->yolo_objects[i]->cloud));
                            }
                        }
                    }
                }
                else
                {
                    ofs << "full_cloud=" << pcd_file << std::endl;
                    if((full_cloud_isSet)&&(!skip_full_cloud))
                    {
                        pcl::io::savePCDFileBinary(data_dir + pcd_file, *(this->full_cloud));
                    }
                    if((objects_cloud_isSet)&&(!skip_objects_cloud))
                    {
                        for(int i = 0; i < this->yolo_objects.size(); i++)
                        {
                            if(this->yolo_objects[i]->cloud->points.size() > 0)
                            {
                                std::stringstream tmp;
                                tmp << data_dir << this->file_name << "_" << i << ".pcd";
                                ofs << i << '_' << this->yolo_objects[i]->name << "=" << tmp.str() << std::endl;

                                pcl::io::savePCDFileBinary(tmp.str(), *(this->yolo_objects[i]->cloud));
                            }
                        }
                    }
                }
            }

            bool load(const std::string &txt_file, const std::string &data_dir, const bool &skip_full_cloud, const bool &skip_objects_cloud)
            {
                std::ifstream ifs(txt_file);

                this->file_name = boost::filesystem::path{txt_file}.stem().string();
                while(!ifs.eof())
                {
                    std::string tmp;
                    ifs >> tmp;

                    std::vector<std::string> strs;
                    boost::split(strs, tmp, boost::is_any_of("="));

                    if(strs.size() < 2) continue;

                    if(strs[0] == "time_stamp")
                    {
                        this->time_stamp = std::chrono::milliseconds(std::stoll(strs[1]));
                    }
                    else if(strs[0] == "full_cloud")
                    {
                        if(!skip_full_cloud)
                        {
                            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                            pcl::io::loadPCDFile(data_dir + strs[1], *cloud);
                            this->full_cloud = cloud;
                        }
                        full_cloud_isSet = !skip_full_cloud;
                    }
                    else
                    {
                        if(!skip_objects_cloud)
                        {
                            std::vector<std::string> strs2;
                            boost::split(strs2, strs[0], boost::is_any_of("_"));
                            boost::shared_ptr<YoloObject<PointT>> yoloObject(new YoloObject<PointT>);
                            yoloObject->name = strs2[1];
                            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                            pcl::io::loadPCDFile(data_dir + strs[1], *cloud);
                            yoloObject->cloud = cloud;
                            this->yolo_objects.push_back(yoloObject);
                        }
                        objects_cloud_isSet = !skip_objects_cloud;
                    }
                }
            }

            bool objectSegmentation(const std::string &tmp_dir,  myClass::objectSegmentation<PointT> object_segmentation, const double &scale)
            {
                std::string txt_file = tmp_dir + this->file_name + ".txt";
                int break_count = 0;
                while(!myFunction::fileExists(txt_file))
                {
                    if(break_count > 100)
                    {
                        std::cerr << txt_file << " not found\n";
                        return false;
                    }
                    break_count++;
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
                for(auto it = lines.begin(); it != lines.end(); ++it)
                //for(int i=0; i < lines.size(); i++)
                {
                    vector<string> strs;
                    boost::split(strs, (*it), boost::is_any_of(" "));
                    //boost::split(strs,lines[i],boost::is_any_of(" "));

                    boost::shared_ptr<YoloObject<PointT>> temp(new YoloObject<PointT>);
                    if(strs.size() == 5)
                    {
                        temp->name = strs[0];
                        
                        object_segmentation.setBound(std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3])*scale, std::stod(strs[4])*scale);
                    }
                    else
                    {
                        for(int i = 0; i < strs.size() - 4; i++)
                        {
                            if(temp->name != "") temp->name += ' ';
                            temp->name += strs[i];
                        }
                        object_segmentation.setBound(std::stod(strs[strs.size() - 4]), std::stod(strs[strs.size() - 3]), std::stod(strs[strs.size() - 2])*scale, std::stod(strs[strs.size() - 1])*scale);
                    }

                    temp->cloud = object_segmentation.division(this->full_cloud);
                    
                    uint8_t r;
                    uint8_t g;
                    uint8_t b;

                    if(!myFunction::name_to_color(temp->name, r, g, b)) continue;

                    temp->cloud = myFunction::fillColor<PointT>(temp->cloud, r, g, b);
                    
                    //boost::shared_ptr<YoloObject<PointT>> temp(new YoloObject<PointT>(strs[0], std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3]), std::stod(strs[4])));
                    if(temp->cloud->points.size() == 0) continue; 
                    yolo_objects.push_back(temp);
                    objects_cloud_isSet = true;
                }
            }

            bool backgroundSegmentation(myClass::backgroundSegmentation<PointT> &background_segmentation)
            {
                this->full_cloud = background_segmentation.compute(this->full_cloud, this->file_name);
                /*for(auto it = this->yolo_objects.begin(); it != this->yolo_objects.end(); ++it)
                {
                     (*it)->cloud = background_segmentation.compute((*it)->cloud, this->file_name);
                }*/
                return true;
            }

            bool backgroundSegmentationYoloObjects(myClass::backgroundSegmentation<PointT> &background_segmentation)
            {
                //this->full_cloud = background_segmentation.compute(this->full_cloud, this->file_name);
                for(auto it = this->yolo_objects.begin(); it != this->yolo_objects.end(); ++it)
                {
                     (*it)->cloud = background_segmentation.compute((*it)->cloud, this->file_name);
                }
                return true;
            }

            bool noiseRemoval(const int meanK = 50, const double StddevMulThresh = 1.0)
            {
                pcl::StatisticalOutlierRemoval<PointT> sor;
                sor.setInputCloud (this->full_cloud);
                sor.setMeanK (meanK);
                sor.setStddevMulThresh (StddevMulThresh);
                sor.filter (*(this->full_cloud));
            }

            bool noiseRemovalYoloObjects(const double percentP = 0.5, const double StddevMulThresh = 1.0)
            {
                pcl::StatisticalOutlierRemoval<PointT> sor;
                for(int i = 0; i < this->yolo_objects.size(); i++)
                {
                    sor.setInputCloud (this->yolo_objects[i]->cloud);
                    sor.setMeanK (this->yolo_objects[i]->cloud->points.size() * percentP);
                    sor.setStddevMulThresh (StddevMulThresh);
                    sor.filter (*(this->yolo_objects[i]->cloud));
                }
            }

            friend ostream& operator<<(ostream &out, CustomFrame &obj)
            {
                out << "time : " << obj.time_stamp.count() << " ms" << std::endl;
                out << "cloud : " << obj.full_cloud->points.size() << " points" << std::endl;

                for(int i = 0; i < obj.yolo_objects.size(); i++)
                {
                    out << "obj" << i << " : " << obj.yolo_objects[i]->name << std::endl;
                }
                return out;
            }

            void show(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const double &size, const bool &show_full_cloud)
            {
                if(show_full_cloud)
                {
                    myFunction::showCloud(viewer, myFunction::XYZ_to_XYZRGB<PointT>(this->full_cloud, false), this->file_name, size);
                }
                else
                {
                    for(int i = 0; i < this->yolo_objects.size(); i++)
                    {
                        //myFunction::showCloudWithText(viewer, obj.yolo_objects[i]->cloud, obj.file_name + std::to_string(i) + obj.yolo_objects[i]->name, obj.yolo_objects[i]->name);
                        myFunction::showCloud(viewer, this->yolo_objects[i]->cloud, this->file_name + std::to_string(i) + this->yolo_objects[i]->name, size);
                    }
                }
            }

            void remove(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const bool &show_full_cloud)
            {
                if(show_full_cloud)
                {
                    myFunction::removeCloud(viewer, this->file_name);
                }
                else
                {
                    for(int i = 0; i < this->yolo_objects.size(); i++)
                    {
                        //myFunction::removeCloudWithText(viewer, obj.file_name + std::to_string(i) + obj.yolo_objects[i]->name);
                        myFunction::removeCloud(viewer, this->file_name + std::to_string(i) + this->yolo_objects[i]->name);
                    }
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

    #pragma endregion CustomFrame

#pragma region loadCustomFrames

	template<typename RandomIt, typename PointT>
	std::vector<boost::shared_ptr<CustomFrame<PointT>>> loadCustomFramesPart(const int &division_num, const std::string &data_dir, const bool &skip_full_cloud, const bool &skip_objects_cloud, const  RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            std::vector<boost::shared_ptr<CustomFrame<PointT>>> customFrames;
			for(auto it = beg; it != end; ++it)
			{
                boost::shared_ptr<CustomFrame<PointT>> customFrame(new CustomFrame<PointT>);
                
                customFrame->load((*it), data_dir, skip_full_cloud, skip_objects_cloud);

                customFrames.push_back(customFrame);
			}
			return customFrames;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, loadCustomFramesPart<RandomIt, PointT>, division_num, data_dir, skip_full_cloud, skip_objects_cloud, beg, mid);
		auto out1 = loadCustomFramesPart<RandomIt, PointT>(division_num, data_dir, skip_full_cloud, skip_objects_cloud, mid, end);
		auto out = handle.get();

		std::copy(out1.begin(), out1.end(), std::back_inserter(out));

		return out;
	}

    template<typename PointT>
	bool loadCustomFrames(const std::string &data_dir, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, bool skip_full_cloud = true, bool skip_objects_cloud = false)
	{
        std::vector<std::string> files;
        for (boost::filesystem::directory_entry & file : boost::filesystem::directory_iterator(data_dir))
        {
            if(file.path().extension().string() == ".txt")
            {
                files.push_back(file.path().string());
            }
        }
        //std::sort(files.begin(), files.end());
        
        int division_num = myFunction::getDivNum<size_t, size_t>(files.size());

        customFrames = loadCustomFramesPart<decltype(files.begin()), PointT>(division_num, data_dir, skip_full_cloud, skip_objects_cloud, files.begin(), files.end());
    }
    
#pragma endregion loadCustomFrames

#pragma region saveCustomFrames

	template<typename RandomIt>
	int saveCustomFramesPart(const int &division_num, const std::string &data_dir, const bool &compress, const bool &skip_full_cloud, const bool &skip_objects_cloud, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->save(data_dir, compress, skip_full_cloud, skip_objects_cloud);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, saveCustomFramesPart<RandomIt>, division_num, data_dir, compress, skip_full_cloud, skip_objects_cloud, beg, mid);
		auto out = saveCustomFramesPart<RandomIt>(division_num, data_dir, compress, skip_full_cloud, skip_objects_cloud, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool saveCustomFrames(std::string &data_dir, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, const bool compress = false, const bool skip_full_cloud = false, const bool skip_objects_cloud = false)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = saveCustomFramesPart(division_num, data_dir, compress, skip_full_cloud, skip_objects_cloud, customFrames.begin(), customFrames.end());
    
        return (count == customFrames.size())? true : false;
    }
    
#pragma endregion saveCustomFrames

#pragma region getCustomFrames

	template<typename PointT>
	bool getCustomFrames(std::string &bagFile, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, std::string &darknet_txt_path, std::string &tmp_dir, const bool postProcessing = true, int number = std::numeric_limits<int>::max())
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

			if(customFrame->set(frameset, bagStartTime, darknet_txt_path, tmp_dir, postProcessing))
			{
				customFrames.push_back(customFrame);
                finished++;
			}
			frameNumber = frameset[0].get_frame_number();
		}
	}

#pragma endregion getCustomFrames

#pragma region backgroundSegmentationCustomFrames

	template<typename RandomIt, typename PointT>
	int backgroundSegmentationCustomFramesPart(const int &division_num, myClass::backgroundSegmentation<PointT> background_segmentation, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->backgroundSegmentation(background_segmentation);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, backgroundSegmentationCustomFramesPart<RandomIt, PointT>, division_num, background_segmentation, beg, mid);
		auto out = backgroundSegmentationCustomFramesPart<RandomIt, PointT>(division_num, background_segmentation, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool backgroundSegmentationCustomFrames(myClass::backgroundSegmentation<PointT> background_segmentation, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = backgroundSegmentationCustomFramesPart<decltype(customFrames.begin()), PointT>(division_num, background_segmentation, customFrames.begin(), customFrames.end());
    }

#pragma endregion backgroundSegmentationCustomFrames

#pragma region backgroundSegmentationCustomFrameYoloObjects

	template<typename RandomIt, typename PointT>
	int backgroundSegmentationCustomFrameYoloObjectsPart(const int &division_num, myClass::backgroundSegmentation<PointT> background_segmentation, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->backgroundSegmentationYoloObjects(background_segmentation);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, backgroundSegmentationCustomFrameYoloObjectsPart<RandomIt, PointT>, division_num, background_segmentation, beg, mid);
		auto out = backgroundSegmentationCustomFrameYoloObjectsPart<RandomIt, PointT>(division_num, background_segmentation, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool backgroundSegmentationCustomFrameYoloObjects(myClass::backgroundSegmentation<PointT> background_segmentation, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = backgroundSegmentationCustomFrameYoloObjectsPart<decltype(customFrames.begin()), PointT>(division_num, background_segmentation, customFrames.begin(), customFrames.end());
    }

#pragma endregion backgroundSegmentationCustomFrameYoloObjects

#pragma region objectSegmentationCustomFrames

	template<typename RandomIt, typename PointT>
	int objectSegmentationCustomFramesPart(const int &division_num, const std::string &tmp_dir, myClass::objectSegmentation<PointT> object_segmentation, const double &scale, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->objectSegmentation(tmp_dir, object_segmentation, scale);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, objectSegmentationCustomFramesPart<RandomIt, PointT>, division_num, tmp_dir, object_segmentation, scale, beg, mid);
		auto out = objectSegmentationCustomFramesPart<RandomIt, PointT>(division_num, tmp_dir, object_segmentation, scale, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool objectSegmentationCustomFrames(const std::string &tmp_dir, myClass::objectSegmentation<PointT> object_segmentation, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, const double &scale = 1.0)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = objectSegmentationCustomFramesPart<decltype(customFrames.begin()), PointT>(division_num, tmp_dir, object_segmentation, scale, customFrames.begin(), customFrames.end());
    }

#pragma endregion objectSegmentationCustomFrames

#pragma region noiseRemovalCustomFrames

	template<typename RandomIt>
	int noiseRemovalCustomFramesPart(const int &division_num, const int &meanK, const double &StddevMulThresh, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->noiseRemoval(meanK, StddevMulThresh);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, noiseRemovalCustomFramesPart<RandomIt>, division_num, meanK, StddevMulThresh, beg, mid);
		auto out = noiseRemovalCustomFramesPart<RandomIt>(division_num, meanK, StddevMulThresh, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool noiseRemovalCustomFrames(std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, const int meanK = 50, const double StddevMulThresh = 1.0)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = noiseRemovalCustomFramesPart(division_num, meanK, StddevMulThresh, customFrames.begin(), customFrames.end());
    }

#pragma endregion noiseRemovalCustomFrames

#pragma region noiseRemovalCustomFrameYoloObjects

	template<typename RandomIt>
	int noiseRemovalCustomFrameYoloObjectsPart(const int &division_num, const double &percentP, const double &StddevMulThresh, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->noiseRemovalYoloObjects(percentP, StddevMulThresh);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, noiseRemovalCustomFrameYoloObjectsPart<RandomIt>, division_num, percentP, StddevMulThresh, beg, mid);
		auto out = noiseRemovalCustomFrameYoloObjectsPart<RandomIt>(division_num, percentP, StddevMulThresh, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool noiseRemovalCustomFrameYoloObjects(std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, const double percentP = 50, const double StddevMulThresh = 1.0)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = noiseRemovalCustomFrameYoloObjectsPart(division_num, percentP, StddevMulThresh, customFrames.begin(), customFrames.end());
    }

#pragma endregion noiseRemovalCustomFrameYoloObjects

}
#endif