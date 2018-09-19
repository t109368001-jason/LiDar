#ifndef CLASS_H_
#define CLASS_H_

#include "../include/function.h"
#include "../include/segmentation.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb_image_write.h"
#include <boost/algorithm/string.hpp>

namespace myClass
{
    class MicroStopwatch
    {
        public:
            int64_t elapsed;
            std::string name;
            boost::posix_time::ptime tictic;
            boost::posix_time::ptime toctoc;
            MicroStopwatch()
            {
                elapsed = 0;
            }
            MicroStopwatch(std::string name)
            {
                elapsed = 0;
                this->name = name;
            }
            void tic()
            {
                tictic = boost::posix_time::microsec_clock::local_time ();
            }
            int64_t toc()
            {
                toctoc = boost::posix_time::microsec_clock::local_time ();
                this->elapsed += (toctoc - tictic).total_microseconds();
                return (toctoc - tictic).total_microseconds();
            }
            std::string toc_string()
            {
                toctoc = boost::posix_time::microsec_clock::local_time ();
                return myFunction::commaFix((toctoc - tictic).total_microseconds());
            }
            void toc_print_string()
            {
                std::cerr << this->name << ": "<< this->toc_string() << " us" <<  std::endl;
            }
            std::string elapsed_string()
            {
                return myFunction::commaFix(elapsed);
            }
            void elapsed_print_string()
            {
                std::cerr << this->name << ": "<< this->elapsed_string() << " us" <<  std::endl;
            }
            int64_t toc_pre()
            {
                return (toctoc - tictic).total_microseconds();
            }
            void clear()
            {
                this->elapsed = 0;
            }
    };

    template<typename PointT>
    class YoloObject
    {
        public:
            double x;
            double y;
            double w;
            double h;
            std::string name;
            typename pcl::PointCloud<PointT>::Ptr cloud;
            YoloObject(std::string name, double x, double y, double w, double h)
            {
                this->name = name;
                this->x = x;
                this->y = y;
                this->w = w;
                this->h = h;
            }
    };
    template<typename PointT>
    class CustomFrame
    {
        public:
            std::chrono::milliseconds timeStamp;
            std::string fileName;
            std::unordered_map<int, std::unordered_set<unsigned long long>> _framesMap;
            typename pcl::PointCloud<PointT>::Ptr cloud;
            bool hasColorFrame;
            std::vector<boost::shared_ptr<YoloObject<PointT>>> yoloObjects;
            bool set(rs2::frameset &frameset, std::chrono::milliseconds startTime, std::string bridge_file)
            //bool set(rs2::frameset &frameset, std::chrono::milliseconds &startTime, std::string &bridge_file, myClass::MicroStopwatch &tt1, myClass::MicroStopwatch &tt2, myClass::MicroStopwatch &tt3, myClass::MicroStopwatch &tt4, myClass::MicroStopwatch &tt5)
            {
                this->hasColorFrame = false;

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
                    this->timeStamp = startTime + std::chrono::milliseconds(int64_t(frameDepth.get_timestamp()));

                    if (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_DEPTH) { frameColor = rs2::colorizer().process(frameColor); }
                    
                    std::string cwd = std::string(getcwd(NULL, 0)) + '/';

                    std::string output_dir = cwd + "tmp/";
                    if(!myFunction::fileExists(output_dir))
                    {
                        mkdir(output_dir.c_str(), 0777);
                    }

                    this->fileName = output_dir + myFunction::millisecondToString(this->timeStamp);
                    
                    std::string png_file = this->fileName + ".png";
                    if(myFunction::fileExists(png_file)) return false;
                    //stbi_write_png( png_file.c_str(), frameColor.get_width(), frameColor.get_height(), frameColor.get_bytes_per_pixel(), frameColor.get_data(), frameColor.get_stride_in_bytes());
                    std::cerr << "png saved: " << png_file << std::endl;

                    //rs2::pointcloud pc;
                    rs2::points points;
                    points = rs2::pointcloud().calculate(frameDepth);

                    this->cloud = myFunction::points_to_pcl<PointT>(points);
                    std::string pcd_file = this->fileName + ".pcd";
                }
                return true;
            }
            
            bool detect(std::string bridge_file)
            {
                std::string png_file = this->fileName + ".png";
                while(1)
                {
                    std::string line;
                    std::ifstream ifs(bridge_file);
                    
                    std::getline(ifs, line);
                    
                    ifs.close();

                    if(line == "")
                    {
                        std::ofstream ofs(bridge_file);

                        ofs.write(png_file.c_str(), png_file.size());
                        ofs.close();
                        break;
                    }

                    std::cerr << "yolo busy..." << '\r' << std::flush;
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                }
                
                std::cerr << "YOLO started" << std::endl;
            }

            bool getTxt(std::string bridge_file)
            {
                std::string txt_file = this->fileName + ".txt";
                std::string png_file = this->fileName + ".png";

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

                    boost::shared_ptr<YoloObject<PointT>> temp(new YoloObject<PointT>(strs[0], std::stod(strs[1]), std::stod(strs[2]), std::stod(strs[3]), std::stod(strs[4])));

                    yoloObjects.push_back(temp);
                }
                std::cerr << yoloObjects.size() << " objects" << std::endl;
            }
            void printObjects()
            {
                int i=1;
                for(auto it = this->yoloObjects.begin(); it != this->yoloObjects.end(); ++it, ++i)
                {
                    std::cerr << i << '.' << (*it)->name << std::endl;
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
}


//*/
                /*
                for (size_t i = 0; i < frameset.size(); i++)
                {
                    rs2::video_frame frameColor = frameset[i].as<rs2::video_frame>();

                    if (frameColor && (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_ANY))
                    {
                        if (frames_map_get_and_set(frameColor.get_profile().stream_type(), frameColor.get_frame_number()))
                        {
                            continue;
                        }

                        rs2::colorizer color_map;
                        if (frameColor.get_profile().stream_type() == rs2_stream::RS2_STREAM_DEPTH)
                        {
                            frameColor = color_map.process(frameColor);
                        }
                        
                        this->frameColorTime = startTime + std::chrono::milliseconds(int64_t(frameColor.get_timestamp()));

                        std::stringstream png_file;
                        std::string cwd = std::string(getcwd(NULL, 0)) + '/';
                        png_file << cwd << "tmp/" << myFunction::millisecondToString(this->frameColorTime) << ".png";
                        
                        std::string dir = cwd + "tmp/";
                        if(!myFunction::fileExists(dir))
                        {
                            mkdir(dir.c_str(), 0777);
                        }
                        std::cerr << png_file.str() << std::endl;
                        //stbi_write_png( png_file.str().c_str(), frameColor.get_width(), frameColor.get_height(), frameColor.get_bytes_per_pixel(), frameColor.get_data(), frameColor.get_stride_in_bytes());
                    }
                }
                //*/
                /*
                for (size_t i = 0; i < frameset.size(); i++)
                {
                    rs2::depth_frame frameDepth = frameset[i].as<rs2::depth_frame>();

                    if (frameDepth && (frameDepth.get_profile().stream_type() == rs2_stream::RS2_STREAM_ANY))
                    {
                        if (frames_map_get_and_set(frameDepth.get_profile().stream_type(), frameDepth.get_frame_number()))
                        {
                            continue;
                        }

                        rs2::pointcloud pc;
                        rs2::points points;
                        points = pc.calculate(frameDepth);

                        this->cloud = myFunction::points_to_pcl<PointT>(points);
                        this->frameDepthTime = startTime + std::chrono::milliseconds(int64_t(frameDepth.get_timestamp()));
                    }
                }
                //*/
#endif