#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <iostream>
#include <future>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include "../include/function.h"

namespace myClass
{
	template<typename PointT>
	class backgroundSegmentation
	{
		private:
			bool isSet;
			typename pcl::PointCloud<PointT>::Ptr background;
			pcl::octree::OctreePointCloudChangeDetector<PointT> *octree;
		public:
			void setBackground(typename pcl::PointCloud<PointT>::Ptr background, double resolution = 1)
			{
				this->background = background;
				
				this->octree = new pcl::octree::OctreePointCloudChangeDetector<PointT>(resolution);
				this->octree->setInputCloud(this->background);
				this->octree->addPointsFromInputCloud();
				this->octree->switchBuffers();

				this->isSet = true;
			}

			template<typename RandomIt, typename RandomIt2>
			RandomIt2 computePart(int division_num, RandomIt2 &points, RandomIt beg, RandomIt end)
			{
				auto len = end - beg;

				if (len < division_num)
				{
					RandomIt2 out;
					for(auto it = beg; it != end; ++it)
					{
						if (static_cast<int> ((*it)->getSize ()) >= 0)
						{
							out.push_back(points[(*it)->getPointIndex()]);
						}
					}
					return out;
				}

				auto mid = beg + len/2;
				auto handle = std::async(std::launch::async, &backgroundSegmentation::computePart<RandomIt, RandomIt2>, this, division_num, points, beg, mid);
				auto out(computePart<RandomIt, RandomIt2>(division_num, points, mid, end));
				auto out1(handle.get());
				
				std::copy(out1.begin(),  out1.end(), std::back_inserter(out));

				return out;
			}

			typename pcl::PointCloud<PointT>::Ptr compute(typename pcl::PointCloud<PointT>::Ptr cloud)
			{
				if(!isSet)
				{
					std::cerr << "Background is not set" << endl;
					return cloud;
				}

				typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
				pcl::octree::OctreePointCloudChangeDetector<PointT> tree = *this->octree;
				std::vector<int> newPointIdxVector;
				
				tree.setInputCloud(cloud);
				tree.addPointsFromInputCloud();
				tree.getPointIndicesFromNewVoxels(newPointIdxVector);
				
				for(auto it = newPointIdxVector.begin(); it != newPointIdxVector.end(); ++it)
				{
					temp->points.push_back(cloud->points[*it]);
				}

				temp->width = (int) temp->points.size();
				temp->height = 1;
				
				return temp;
			}
			
	};

	template<typename PointT>
	class objectSegmentation
	{
		public:
			string camera_O;
			double camera_Height;
			double camera_Width;
			//double camera_Vertical_FOV;
			//double camera_Horizontal_FOV;
			double camera_FOV;
			//double camera_Vertical_Depth;
			//double camera_Horizontal_Depth;
			double camera_Depth;
			bool cameraParameterIsSet;

			double LiDar_Height_Offset;
			double LiDar_Horizontal_Offset;
			double focal_Leftength_Offset;

			double LiDar_Up_Angle;
			double LiDar_Down_Angle;
			double LiDar_Left_Angle;
			double LiDar_Right_Angle;
			bool BoundIsSet;

			bool keep_Inside;

			objectSegmentation()
			{
				cameraParameterIsSet = false;
				BoundIsSet = false;
			}

			objectSegmentation(string camera_O, double camera_Height, double camera_Width, double camera_Vertical_FOV, double camera_Horizontal_FOV)
			{
				setCameraParameter(camera_O, camera_Height, camera_Width, camera_Vertical_FOV, camera_Horizontal_FOV);
				BoundIsSet = false;
			}

			//int setCameraParameter(string camera_O, double camera_Height, double camera_Width, double camera_Vertical_FOV, double camera_Horizontal_FOV)
			int setCameraParameter(string camera_O, double camera_Width, double camera_Height, double camera_FOV)
			{
				this->camera_O = camera_O;
				this->camera_Width = camera_Width;
				this->camera_Height = camera_Height;
				//this->camera_Horizontal_FOV = camera_Horizontal_FOV;
				//this->camera_Vertical_FOV = atan((camera_Height/2.0)/((camera_Width/2.0)/tan(camera_Horizontal_FOV/2.0)))*2.0;
				//this->camera_Vertical_Depth = (camera_Height/2.0) / tan(camera_Vertical_FOV/2.0);		//計算深度, 計算用
				//this->camera_Horizontal_Depth = (camera_Width/2.0) / tan(camera_Horizontal_FOV/2.0);		//計算深度, 計算用
				this->camera_FOV = camera_FOV;
				this->camera_Depth = (camera_Width/2.0) / tan(camera_FOV/2.0);
				cameraParameterIsSet = true;
				BoundIsSet = false;				//須重新計算邊界
				return 0;
			}

			int setLiDarParameter(double LiDar_Horizontal_Offset = 0, double LiDar_Height_Offset = 0, double focal_Leftength_Offset = 0)
			{
				if(std::fabs(LiDar_Horizontal_Offset) > M_PI)
				{
					std::cerr << "LiDar Horizontal Offset should be -M_PI ~ M_PI" << endl;
					return -1;
				}
				this->LiDar_Horizontal_Offset = LiDar_Horizontal_Offset;
				this->LiDar_Height_Offset = LiDar_Height_Offset;
				this->focal_Leftength_Offset = focal_Leftength_Offset;

				BoundIsSet = false;				//須重新計算邊界
				return 0;
			}

			int setBound(double object_X, double object_Y, double object_Height, double object_Width)
			{
				if(!cameraParameterIsSet)
				{
					std::cerr << "Camera parameter is not set" << endl;
					return -1;
				}
				double object_X_Temp = object_X;
				double object_Y_Temp = object_Y;
				
				//YOLO原點修正成OpenGL原點
				if(this->camera_O == "UR")
				{
					object_X_Temp = this->camera_Width - object_X_Temp;
				}
				else if(this->camera_O == "DR")
				{
					object_X_Temp = this->camera_Width - object_X_Temp;
					object_Y_Temp = this->camera_Height - object_Y_Temp;
				}
				else if(this->camera_O == "DL")
				{
					object_Y_Temp = this->camera_Height - object_Y_Temp;
				}
				
				//將物件原點改為中心, 右邊為+x, 上面為+y
				double object_Up = -(object_Y_Temp - this->camera_Height/2);
				double object_Down = object_Up - object_Height;
				double object_Left = object_X_Temp - this->camera_Width/2;
				double object_Right = object_Left + object_Width;
				
				//將邊界轉換為角度
				//double object_Up_Angle = atan(object_Up / this->camera_Vertical_Depth);
				//double object_Down_Angle = atan(object_Down / this->camera_Vertical_Depth);
				//double object_Left_Angle = atan(object_Left / this->camera_Horizontal_Depth);
				//double object_Right_Angle = atan(object_Right / this->camera_Horizontal_Depth);
				double object_Up_Angle = atan(object_Up / this->camera_Depth);
				double object_Down_Angle = atan(object_Down / this->camera_Depth);
				double object_Left_Angle = atan(object_Left / this->camera_Depth);
				double object_Right_Angle = atan(object_Right / this->camera_Depth);
				
				//將邊界的角度以LiDar的高低差和最佳化焦距修正, 修正後的值在等於焦距時無誤差
				if(this->LiDar_Height_Offset == 0)
				{
					this->LiDar_Up_Angle = object_Up_Angle;
					this->LiDar_Down_Angle = object_Down_Angle;
				}
				else
				{
					this->LiDar_Up_Angle = atan((this->focal_Leftength_Offset * tan(object_Up_Angle) + this->LiDar_Height_Offset) / this->focal_Leftength_Offset);
					this->LiDar_Down_Angle = atan((this->focal_Leftength_Offset * tan(object_Down_Angle) + this->LiDar_Height_Offset) / this->focal_Leftength_Offset);
				}
				this->LiDar_Left_Angle = object_Left_Angle;
				this->LiDar_Right_Angle = object_Right_Angle;

				//修正角度, 水平右-90 左90 垂直上0 下180
				this->LiDar_Up_Angle = M_PI_2 - this->LiDar_Up_Angle;
				this->LiDar_Down_Angle = M_PI_2 - this->LiDar_Down_Angle;
				this->LiDar_Left_Angle = -this->LiDar_Left_Angle;
				this->LiDar_Right_Angle = -this->LiDar_Right_Angle;
				//修正LiDar水平原點誤差
				this->LiDar_Left_Angle += this->LiDar_Horizontal_Offset;
				this->LiDar_Right_Angle += this->LiDar_Horizontal_Offset;

				if(this->LiDar_Left_Angle < this->LiDar_Right_Angle)
				{
					this->LiDar_Left_Angle += M_PI;
					this->LiDar_Right_Angle -= M_PI;
				}
				if(this->LiDar_Left_Angle > M_PI)
				{
					this->LiDar_Left_Angle -= M_PI*2;
				}
				if(this->LiDar_Right_Angle < -M_PI)
				{
					this->LiDar_Right_Angle += M_PI*2;
				}

				BoundIsSet = true;
				return 0;
			}

			void printAngle()
			{
				std::cerr << "LiDar_Up_Angle: " << this->LiDar_Up_Angle << std::endl;
				std::cerr << "LiDar_Down_Angle: " << this->LiDar_Down_Angle << std::endl;
				std::cerr << "LiDar_Left_Angle: " << this->LiDar_Left_Angle << std::endl;
				std::cerr << "LiDar_Right_Angle: " << this->LiDar_Right_Angle << std::endl;
			}

			template<typename RandomIt>
			std::vector<PointT, Eigen::aligned_allocator<PointT>> divisionPart(int division_num, RandomIt beg, RandomIt end)
			{
				auto len = end - beg;

				if (len < division_num)
				{
					std::vector<PointT, Eigen::aligned_allocator<PointT>> out;
					for(auto it = beg; it != end; ++it)
					{
						if((!this->keep_Inside) ^ this->pointIsInside((*it)))
						{
							out.push_back(*it);
						}
					}
					return out;
				}

				auto mid = beg + len/2;
				auto handle = std::async(std::launch::async, &objectSegmentation::divisionPart<RandomIt>, this, division_num, beg, mid);
				auto out(divisionPart<RandomIt>(division_num, mid, end));
				auto out1(handle.get());
				
				std::copy(out1.begin(),  out1.end(), std::back_inserter(out));

				return out;
			}

			typename pcl::PointCloud<PointT>::Ptr division(typename pcl::PointCloud<PointT>::Ptr input, bool keep_Inside = true)
			{
				if(!BoundIsSet)
				{
					std::cerr << "\nBound parameter is not set\n";
					return input;
				}
				if(((LiDar_Up_Angle - LiDar_Down_Angle) == 0)||((LiDar_Right_Angle - LiDar_Left_Angle) == 0))
				{
					std::cerr << "\nInput angle valid\n";
					return input;
				}

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;
				
				int division_num = std::ceil(input->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
				
				cloud->points = divisionPart(division_num, input->points.begin(), input->points.end());

				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
				return cloud;
			}

			typename pcl::PointCloud<PointT>::Ptr easyDivision(typename pcl::PointCloud<PointT>::Ptr input, double H_FOV, double offset = 0, bool keep_Inside = true)
			{
				if(this->setCameraParameter("UL", 16, 9, H_FOV) == -1) return input;
				if(this->setLiDarParameter(offset) == -1) return input;
				if(this->setBound(0, 0, 1, 1) == -1) return input;

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;
				
				int division_num = std::ceil(input->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
				
				cloud->points = divisionPart(division_num, input->points.begin(), input->points.end());
				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
				return cloud;
			}

		private:
			bool pointIsInside(PointT point)
			{
				//計算point向量之水平及垂直角度
				double point_Vertical_Angle = acos(point.z / myFunction::norm(point));
				double point_Horizontal_Angle = atan(point.y / point.x);

				//修正水平角度, 因 -M_PI_2 <= atan <= M_PI_2  >> 
				if (point.x < 0.0)
				{
					if(point.y < 0.0)
					{
						point_Horizontal_Angle -= M_PI;
					}
					else
					{
						point_Horizontal_Angle += M_PI;
					}
				}

				//檢查是否超出邊界
				if(this->LiDar_Right_Angle > this->LiDar_Left_Angle)
				{
					if((point_Horizontal_Angle < this->LiDar_Right_Angle)&&(point_Horizontal_Angle > this->LiDar_Left_Angle)){ return false; }
				}
				else
				{
					if(point_Horizontal_Angle < this->LiDar_Right_Angle){ return false; }
					if(point_Horizontal_Angle > this->LiDar_Left_Angle){ return false; }
				}
				if(point_Vertical_Angle < this->LiDar_Up_Angle){ return false; }
				if(point_Vertical_Angle > this->LiDar_Down_Angle){ return false; }

				return true;
			}
	};
}
#endif