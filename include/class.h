#ifndef CLASS_H_
#define CLASS_H_

#include "../include/function.h"
#include "../include/segmentation.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb_image_write.h"
#include <boost/algorithm/string.hpp>

namespace myClass
{
    class YoloObject
    {
        public:
            double x;
            double y;
            double w;
            double h;
            std::string name;
    };
    template<typename PointT>
    class CustomFrame
    {
        public:
            std::chrono::milliseconds time;
            std::unordered_map<int, std::unordered_set<unsigned long long>> _framesMap;
            typename pcl::PointCloud<PointT>::Ptr cloud;
            bool isSet;
            CustomFrame(rs2::frameset &frameset, std::chrono::milliseconds timestamp)
            {
                auto frameDepth = frameset.get_depth_frame();
                auto frameColor = frameset.get_color_frame();
                
                //if(frameColor)
                //for (size_t i = 0; i < frameset.size(); i++) {
                    //rs2::video_frame frameColor = frameset[i].as<rs2::video_frame>();

                    if (frameColor){// && (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_ANY)) {
                        if (frames_map_get_and_set(frameColor.get_profile().stream_type(), frameColor.get_frame_number())) {
                            return;
                        }

                        rs2::colorizer color_map;
                        if (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_DEPTH) {
                            frameColor = color_map.process(frameColor);
                        }
                        
                        std::stringstream png_file;
                        std::string cwd = std::string(getcwd(NULL, 0)) + '/';
                        png_file << cwd << "tmp/" << myFunction::millisecondToString(this->time) << ".png";
                        
                        std::string dir = cwd + "tmp/";
                        if(!myFunction::fileExists(dir))
                        {
                            mkdir(dir.c_str(), 0777);
                        }
                        std::cerr << png_file.str() << std::endl;
                        stbi_write_png( png_file.str().c_str(), frameColor.get_width(), frameColor.get_height(), 
                                        frameColor.get_bytes_per_pixel(), frameColor.get_data(), frameColor.get_stride_in_bytes());
                    }
                //}
                
                if (frameDepth) {
                    if (frames_map_get_and_set(rs2_stream::RS2_STREAM_ANY, frameDepth.get_frame_number())) {
                        return;
                    }
                    
                    rs2::pointcloud pc;
                    rs2::points points;
                    points = pc.calculate(frameDepth);

                    this->cloud = myFunction::points_to_pcl<PointT>(points);
                }
                this->time = timestamp;
                this->isSet = true;
            }
            /*
            bool getTxt()
            {
                std::stringstream txt_file;
                std::string cwd = std::string(getcwd(NULL, 0)) + '/';

                txt_file << cwd << "tmp/" << myFunction::millisecondToString(this->time) << ".txt";
                
                std::ifstream ifs;
                ifs.open(txt_file);
                std::string line;
                
                if(!ifs.is_open()) return false;
                while(!ifs.eof())
                {
                    std::getline(ifs, line);
                    
                    if(line == "") continue;

                    std::vector<std::string> ss;

                    boost::algorithm::split(ss, line, boost::algorithm::is_any_of(' '));
                    for(auto it = ss.begin(); it != ss.end(); ++it)
                    {
                        std::cerr << *it << std::endl;
                    }
                }
                if(line == "")
                {
                    return false;
                }
                else
                {
                    
                }
            }
            */
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
}
#endif