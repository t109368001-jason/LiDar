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

#pragma region CustomFrame

    template<typename PointT>
    class CustomFrame
    {
        public:
            std::string file_name;
            std::chrono::milliseconds time_stamp;
            typename pcl::PointCloud<PointT>::Ptr entire_cloud;
            std::vector<boost::shared_ptr<YoloObject<PointT>>> yolo_objects;

            bool entire_cloud_isSet;

            bool set(rs2::frameset &frameset, const std::chrono::milliseconds &startTime, const std::string &bridge_file, const std::string &tmp_dir)
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

                    std::string abs_tmp_dir = cwd + tmp_dir;
                    
                    this->file_name = myFunction::millisecondToString(this->time_stamp);
                    
                    std::string png_file = abs_tmp_dir + this->file_name + ".png";
                    while(myFunction::fileExists(png_file))
                    {
                        this->time_stamp += std::chrono::milliseconds(33);
                        this->file_name = myFunction::millisecondToString(this->time_stamp);
                    
                        png_file = abs_tmp_dir + this->file_name + ".png";
                    }
                    stbi_write_png( png_file.c_str(), frameColor.get_width(), frameColor.get_height(), frameColor.get_bytes_per_pixel(), frameColor.get_data(), frameColor.get_stride_in_bytes());
                    
                    this->detect(png_file, bridge_file);

                    /////////////////////////////////////////////////////////////////////////
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
                    ////////////////////////////////////////////////////////////////////////*/


                    rs2::points points;
                    points = rs2::pointcloud().calculate(frameDepth);

                    this->entire_cloud = myFunction::points_to_pcl<PointT>(points);
                    entire_cloud_isSet = true;
                }
                return true;
            }
            
            bool save(const std::string &data_dir, const bool &compress)
            {
                std::string file_name = data_dir + this->file_name;
                std::string txt_file = file_name + ".txt";
                std::string pcd_file = file_name + ".pcd";

                std::ofstream ofs(txt_file);
                ofs << "time_stamp=" << this->time_stamp.count() << std::endl;
                if(compress)
                {
                    if(entire_cloud_isSet)
                    {
                        ofs << "entire_cloud=" << pcd_file << std::endl;
                        pcl::io::savePCDFileBinaryCompressed(pcd_file, *(this->entire_cloud));
                    }
                    for(int i = 0; i < this->yolo_objects.size(); i++)
                    {
                        std::stringstream tmp;
                        tmp << file_name << "_" << i << ".pcd";
                        ofs << i << '_' << this->yolo_objects[i]->name << "=" << tmp.str() << std::endl;

                        pcl::io::savePCDFileBinaryCompressed(tmp.str(), *(this->yolo_objects[i]->cloud));
                    }
                }
                else
                {
                    ofs << "entire_cloud=" << pcd_file << std::endl;
                    if(entire_cloud_isSet)
                    {
                        pcl::io::savePCDFileBinary(pcd_file, *(this->entire_cloud));
                    }
                    for(int i = 0; i < this->yolo_objects.size(); i++)
                    {
                        std::stringstream tmp;
                        tmp << file_name << "_" << i << ".pcd";
                        ofs << i << '_' << this->yolo_objects[i]->name << "=" << tmp.str() << std::endl;

                        pcl::io::savePCDFileBinary(tmp.str(), *(this->yolo_objects[i]->cloud));
                    }
                }
            }

            bool load(const std::string &txt_file, const bool &skip_entire_cloud)
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
                    else if(strs[0] == "entire_cloud")
                    {
                        if(!skip_entire_cloud)
                        {
                            typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                            pcl::io::loadPCDFile(strs[1], *cloud);
                            this->entire_cloud = cloud;
                        }
                        entire_cloud_isSet = !skip_entire_cloud;
                    }
                    else
                    {
                        std::vector<std::string> strs2;
                        boost::split(strs2, strs[0], boost::is_any_of("_"));
                        boost::shared_ptr<YoloObject<PointT>> yoloObject(new YoloObject<PointT>);
                        yoloObject->name = strs2[1];
                        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                        pcl::io::loadPCDFile(strs[1], *cloud);
                        yoloObject->cloud = cloud;
                        this->yolo_objects.push_back(yoloObject);
                    }
                }
            }

            bool objectSegmentation(const std::string &tmp_dir,  myClass::objectSegmentation<PointT> object_segmentation)
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
                    temp->name = strs[0];
                    
                    object_segmentation.setBound(std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3]), std::stod(strs[4]));
                    temp->cloud = object_segmentation.division(this->entire_cloud);
                    
                    uint8_t r;
                    uint8_t g;
                    uint8_t b;

                    myFunction::name_to_color(temp->name, r, g, b);

                    temp->cloud = myFunction::fillColor<PointT>(temp->cloud, r, g, b);
                    
                    //boost::shared_ptr<YoloObject<PointT>> temp(new YoloObject<PointT>(strs[0], std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3]), std::stod(strs[4])));
                    if(temp->cloud->points.size()) yolo_objects.push_back(temp);
                }
            }

            bool backgroundSegmentation(myClass::backgroundSegmentation<PointT> &background_segmentation)
            {
                this->entire_cloud = background_segmentation.compute(this->entire_cloud, this->file_name);
                /*for(auto it = this->yolo_objects.begin(); it != this->yolo_objects.end(); ++it)
                {
                     (*it)->cloud = background_segmentation.compute((*it)->cloud, this->file_name);
                }*/
                return true;
            }

            bool noiseRemoval(const int meanK = 50, const double StddevMulThresh = 1.0)
            {
                pcl::StatisticalOutlierRemoval<PointT> sor;
                sor.setInputCloud (this->entire_cloud);
                sor.setMeanK (meanK);
                sor.setStddevMulThresh (StddevMulThresh);
                sor.filter (*(this->entire_cloud));
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

            friend boost::shared_ptr<pcl::visualization::PCLVisualizer> operator<<(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, CustomFrame &obj)
            {
                for(int i = 0; i < obj.yolo_objects.size(); i++)
                {
                    myFunction::showCloudWithText(viewer, obj.yolo_objects[i]->cloud, obj.file_name + std::to_string(i) + obj.yolo_objects[i]->name, obj.yolo_objects[i]->name);
                }
                return viewer;
            }

            friend boost::shared_ptr<pcl::visualization::PCLVisualizer> operator-=(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, CustomFrame &obj)
            {
                for(int i = 0; i < obj.yolo_objects.size(); i++)
                {
                    myFunction::removeCloudWithText(viewer, obj.file_name + std::to_string(i) + obj.yolo_objects[i]->name);
                }
                return viewer;
            }

        private:
            std::unordered_map<int, std::unordered_set<unsigned long long>> _framesMap;

            bool detect(const std::string &png_file, const std::string &bridge_file)
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

    #pragma endregion CustomFrame

#pragma region loadCustomFrames

	template<typename RandomIt, typename PointT>
	std::vector<boost::shared_ptr<CustomFrame<PointT>>> loadCustomFramesPart(const int &division_num, const bool &skip_entire_cloud, const  RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            std::vector<boost::shared_ptr<CustomFrame<PointT>>> customFrames;
			for(auto it = beg; it != end; ++it)
			{
                boost::shared_ptr<CustomFrame<PointT>> customFrame(new CustomFrame<PointT>);
                
                customFrame->load((*it), skip_entire_cloud);

                customFrames.push_back(customFrame);
			}
			return customFrames;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, loadCustomFramesPart<RandomIt, PointT>, division_num, skip_entire_cloud, beg, mid);
		auto out1 = loadCustomFramesPart<RandomIt, PointT>(division_num, skip_entire_cloud, mid, end);
		auto out = handle.get();

		std::copy(out1.begin(), out1.end(), std::back_inserter(out));

		return out;
	}

    template<typename PointT>
	bool loadCustomFrames(const std::string &data_dir, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, bool skip_entire_cloud = true)
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

        customFrames = loadCustomFramesPart<decltype(files.begin()), PointT>(division_num, skip_entire_cloud, files.begin(), files.end());
    }
    
#pragma endregion loadCustomFrames

#pragma region saveCustomFrames

	template<typename RandomIt>
	int saveCustomFramesPart(const int &division_num, const std::string &data_dir, const bool &compress, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->save(data_dir, compress);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, saveCustomFramesPart<RandomIt>, division_num, data_dir, compress, beg, mid);
		auto out = saveCustomFramesPart<RandomIt>(division_num, data_dir, compress, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool saveCustomFrames(std::string &data_dir, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames, const bool compress = false)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = saveCustomFramesPart(division_num, data_dir, compress, customFrames.begin(), customFrames.end());
    
        return (count == customFrames.size())? true : false;
    }
    
#pragma endregion saveCustomFrames

#pragma region getCustomFrames

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

#pragma region backgroundSegmentationCustomFrames

	template<typename RandomIt, typename PointT>
	int objectSegmentationCustomFramesPart(const int &division_num, const std::string &tmp_dir, myClass::objectSegmentation<PointT> object_segmentation, const RandomIt &beg, const RandomIt &end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
            int out;
			for(auto it = beg; it != end; ++it)
			{
                (*it)->objectSegmentation(tmp_dir, object_segmentation);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, objectSegmentationCustomFramesPart<RandomIt, PointT>, division_num, tmp_dir, object_segmentation, beg, mid);
		auto out = objectSegmentationCustomFramesPart<RandomIt, PointT>(division_num, tmp_dir, object_segmentation, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

    template<typename PointT>
	bool objectSegmentationCustomFrames(const std::string &tmp_dir, myClass::objectSegmentation<PointT> object_segmentation, std::vector<boost::shared_ptr<CustomFrame<PointT>>> &customFrames)
	{
        int division_num = myFunction::getDivNum<size_t, size_t>(customFrames.size());

        int count = objectSegmentationCustomFramesPart<decltype(customFrames.begin()), PointT>(division_num, tmp_dir, object_segmentation, customFrames.begin(), customFrames.end());
    }

#pragma endregion backgroundSegmentationCustomFrames

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

}
#endif