#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <iostream>
#include <future>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include "../include/function.h"

namespace myClass
{
#pragma region backgroundSegmentation

	template<typename PointT>
	class backgroundSegmentation
	{
		private:
			bool isSet;
			typename pcl::PointCloud<PointT>::Ptr background;
			pcl::octree::OctreePointCloudChangeDetector<PointT> *octree;
		public:
			void setBackground(const typename pcl::PointCloud<PointT>::Ptr &background, const double &resolution = 1)
			{
				this->background = background;
				
				this->octree = new pcl::octree::OctreePointCloudChangeDetector<PointT>(resolution);
				this->octree->setInputCloud(this->background);
				this->octree->addPointsFromInputCloud();
				this->octree->switchBuffers();

				this->isSet = true;
			}

			template<typename RandomIt, typename RandomIt2>
			RandomIt2 computePart(const int &division_num, const RandomIt2 &points, const RandomIt &beg, const RandomIt &end)
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

			typename pcl::PointCloud<PointT>::Ptr compute(const typename pcl::PointCloud<PointT>::Ptr &cloud)
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

#pragma endregion backgroundSegmentation
	
#pragma region objectSegmentation

	template<typename PointT>
	class objectSegmentation
	{
		public:
			string camera_O;
			
			double camera_Height;
			double camera_Width;
			double camera_FOV;
			double camera_Vertical_FOV;
			//double camera_Horizontal_FOV;
			double depth_Camera_FOV;
			double camera_Depth;
			double camera_Vertical_Depth;
			//double camera_Horizontal_Depth;
			bool cameraParameterIsSet;

			double depth_Camera_Width;
			double depth_Camera_Height;
			double depth_Camera_OffsetX;
			double depth_Camera_OffsetY;
			bool depthCameraParameterIsSet;

			double LiDar_Height_Offset;
			double LiDar_Horizontal_Offset;
			double focal_Leftength_Offset;

			double LiDar_Up_Angle;
			double LiDar_Down_Angle;
			double LiDar_Left_Angle;
			double LiDar_Right_Angle;
			bool BoundIsSet;

			bool keep_Inside;

			double hmin;
			double hmax;
			double vmin;
			double vmax;

			objectSegmentation()
			{
				this->hmin = std::numeric_limits<double>::max();
				this->hmax = std::numeric_limits<double>::min();
				this->vmin = std::numeric_limits<double>::max();
				this->vmax = std::numeric_limits<double>::min();
				this->LiDar_Height_Offset = 0;
				this->LiDar_Horizontal_Offset = 0;
				this->focal_Leftength_Offset = 0;
				this->cameraParameterIsSet = false;
				this->BoundIsSet = false;
			}

			objectSegmentation(string camera_O, double camera_Height, double camera_Width, double camera_Vertical_FOV, double camera_Horizontal_FOV)
			{
				setCameraParameter(camera_O, camera_Height, camera_Width, camera_Vertical_FOV, camera_Horizontal_FOV);
				
				this->hmin = std::numeric_limits<double>::max();
				this->hmax = std::numeric_limits<double>::min();
				this->vmin = std::numeric_limits<double>::max();
				this->vmax = std::numeric_limits<double>::min();
				this->LiDar_Height_Offset = 0;
				this->LiDar_Horizontal_Offset = 0;
				this->focal_Leftength_Offset = 0;
				this->BoundIsSet = false;
			}

			//int setCameraParameter(string camera_O, double camera_Height, double camera_Width, double camera_Vertical_FOV, double camera_Horizontal_FOV)
			int setCameraParameter(string camera_O, double camera_Width, double camera_Height, double camera_FOV, double depth_Camera_FOV = 0)
			{
				this->camera_O = camera_O;
				this->camera_Width = camera_Width;
				this->camera_Height = camera_Height;
				//this->camera_Horizontal_FOV = camera_Horizontal_FOV;
				//this->camera_Vertical_FOV = atan((camera_Height/2.0)/((camera_Width/2.0)/tan(camera_Horizontal_FOV/2.0)))*2.0;
				//this->camera_Vertical_Depth = (camera_Height/2.0) / tan(camera_Vertical_FOV/2.0);		//計算深度, 計算用
				//this->camera_Horizontal_Depth = (camera_Width/2.0) / tan(camera_Horizontal_FOV/2.0);		//計算深度, 計算用
				this->camera_FOV = camera_FOV;
				this->depth_Camera_FOV = depth_Camera_FOV;
				this->camera_Depth = (camera_Width/2.0) / tan(camera_FOV/2.0);

				
				if(this->depth_Camera_FOV != 0)
				{
					this->depth_Camera_Width = this->camera_Depth * tan(this->depth_Camera_FOV / 2.0) * 2.0;
					this->depth_Camera_Height = this->depth_Camera_Width * this->camera_Height / this->camera_Width;
					this->depth_Camera_OffsetX = (this->camera_Width - this->depth_Camera_Width) / 2.0;
					this->depth_Camera_OffsetY = (this->camera_Height - this->depth_Camera_Height) / 2.0;;
					this->depthCameraParameterIsSet = true;
				}


				this->cameraParameterIsSet = true;
				this->BoundIsSet = false;				//須重新計算邊界
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

				this->BoundIsSet = false;				//須重新計算邊界
				return 0;
			}

			int setBound(double object_X, double object_Y, double object_Width, double object_Height)
			{
				if(!this->cameraParameterIsSet)
				{
					std::cerr << "Camera parameter is not set" << endl;
					return -1;
				}

				double object_X_Temp = object_X;
				double object_Y_Temp = object_Y;
				

				if(this->depthCameraParameterIsSet)
				{
					object_X_Temp += this->depth_Camera_OffsetX;
					object_Y_Temp += this->depth_Camera_OffsetY;
					object_Width *= this->depth_Camera_Width / this->camera_Width;
					object_Height *= this->depth_Camera_Height / this->camera_Height;
				}

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

				this->BoundIsSet = true;
				return 0;
			}

			void printAngle()
			{
				std::cerr << "LiDar_Up_Angle: " << this->LiDar_Up_Angle/M_PI*180.0 << std::endl;
				std::cerr << "LiDar_Down_Angle: " << this->LiDar_Down_Angle/M_PI*180.0 << std::endl;
				std::cerr << "LiDar_Left_Angle: " << this->LiDar_Left_Angle/M_PI*180.0 << std::endl;
				std::cerr << "LiDar_Right_Angle: " << this->LiDar_Right_Angle/M_PI*180.0 << std::endl;
			}

			template<typename RandomIt>
			std::vector<PointT, Eigen::aligned_allocator<PointT>> divisionPart(const int &division_num, const RandomIt &beg, const RandomIt &end)
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

			typename pcl::PointCloud<PointT>::Ptr division(const typename pcl::PointCloud<PointT>::Ptr &input, const bool &keep_Inside = true)
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
					std::cerr << "this->hmin: " << this->hmin/M_PI*180.0 << std::endl;
					std::cerr << "this->hmax: " << this->hmax/M_PI*180.0 << std::endl;
					std::cerr << "this->vmin: " << this->vmin/M_PI*180.0 << std::endl;
					std::cerr << "this->vmax: " << this->vmax/M_PI*180.0 << std::endl << std::endl;
				return cloud;
			}

			typename pcl::PointCloud<PointT>::Ptr easyDivision(const typename pcl::PointCloud<PointT>::Ptr &input, const double &H_FOV, const double &offset = 0, const bool &keep_Inside = true)
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
			bool pointIsInside(const PointT &point)
			{
				double point_Vertical_Angle = 0;
				double point_Horizontal_Angle = 0;
				//計算point向量之水平及垂直角度
				if(this->depthCameraParameterIsSet)	//x -> z; y -> -x z -> -y;
				{
					point_Vertical_Angle = acos((-point.y) / myFunction::norm(point));
					point_Horizontal_Angle = atan((-point.x) / point.z);
					this->hmin = std::fmin(point_Horizontal_Angle, this->hmin);
					this->hmax = std::fmax(point_Horizontal_Angle, this->hmax);
					this->vmin = std::fmin(point_Vertical_Angle, this->vmin);
					this->vmax = std::fmax(point_Vertical_Angle, this->vmax);
				}
				else
				{
					point_Vertical_Angle = acos(point.z / myFunction::norm(point));
					point_Horizontal_Angle = atan(point.y / point.x);
				}

				//修正水平角度, 因 -M_PI_2 <= atan <= M_PI_2  >> 
				if(this->depthCameraParameterIsSet)	//x -> z; y -> -x z -> -y;
				{
					if (point.z < 0.0)
					{
						if((-point.x) < 0.0)
						{
							point_Horizontal_Angle -= M_PI;
						}
						else
						{
							point_Horizontal_Angle += M_PI;
						}
					}
				}
				else
				{
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
				}

				//檢查是否超出邊界
				if(this->LiDar_Right_Angle > this->LiDar_Left_Angle)
				{
					if((point_Horizontal_Angle < this->LiDar_Right_Angle)&&(point_Horizontal_Angle > this->LiDar_Left_Angle)){ return false; }
				}
				else
				{
					if(point_Horizontal_Angle < this->LiDar_Right_Angle){std::cerr << "R: " << point_Horizontal_Angle/M_PI*180.0 << std::endl; return false; }
					if(point_Horizontal_Angle > this->LiDar_Left_Angle){std::cerr << "L: " << point_Horizontal_Angle/M_PI*180.0 << std::endl; return false; }
				}
				if(point_Vertical_Angle < this->LiDar_Up_Angle){std::cerr << "U: " << point_Vertical_Angle/M_PI*180.0 << std::endl; return false; }
				if(point_Vertical_Angle > this->LiDar_Down_Angle){std::cerr << "D: " << point_Vertical_Angle/M_PI*180.0 << std::endl; return false; }

				if(std::fabs(point.x*point.y*point.z) == 0.0){ return false; }

				return true;
			}
	};

#pragma endregion objectSegmentation

	template<typename PointT>
	class objectSegmentation2
	{
		public:
			string camera_O;
			
			double camera_Width;
			double camera_Height;
			double camera_Depth;
			bool cameraParameterIsSet;


			double segmentation_Up_Bound;
			double segmentation_Down_Bound;
			double segmentation_Left_Bound;
			double segmentation_Right_Bound;
			double segmentation_Scale_Fix;

			double Up_Bound;
			double Down_Bound;
			double Left_Bound;
			double Right_Bound;

			bool BoundIsSet;

			bool keep_Inside;

			objectSegmentation2()
			{
				this->BoundIsSet = false;
				this->cameraParameterIsSet = false;
			}

			objectSegmentation2(const string &camera_O, const double &camera_Width, const double &camera_Height, const double &camera_FOV, double depth_Camera_FOV = 0)
			{
				setCameraParameter(camera_O, camera_Width, camera_Height, camera_FOV, depth_Camera_FOV);
			}

			bool setCameraParameter(const string &camera_O, const double &camera_Width, const double &camera_Height, const double &camera_FOV, double depth_Camera_FOV = 0)
			{
				this->camera_O = camera_O;
				this->camera_Width = camera_Width;
				this->camera_Height = camera_Height;
				this->camera_Depth = (camera_Width/2.0) / tan(camera_FOV/2.0);

				if(depth_Camera_FOV != 0)
				{
					this->segmentation_Scale_Fix = (2 * this->camera_Depth * tan(depth_Camera_FOV / 2.0)) / camera_Width;
				}
				else
				{
					this->segmentation_Scale_Fix = 1.0;
				}

				this->cameraParameterIsSet = true;
				this->BoundIsSet = false;				//須重新計算邊界
				return true;
			}

			//x and y is center
			bool setBound(const double &object_X, const double &object_Y, const double &object_Width, const double &object_Height)
			{
				if(!this->cameraParameterIsSet)
				{
					std::cerr << "Camera parameter is not set" << endl;
					return false;
				}
				double object_X_Temp = object_X;
				double object_Y_Temp = object_Y;

				if(this->camera_O == "UL")
				{
					object_X_Temp -= this->camera_Width / 2.0;
					object_Y_Temp -= this->camera_Height / 2.0;
				}

				this->segmentation_Left_Bound = (object_X_Temp - (object_Width/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Right_Bound = (object_X_Temp + (object_Width/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Up_Bound = (object_Y_Temp - (object_Height/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Down_Bound = (object_Y_Temp + (object_Height/2.0)) * this->segmentation_Scale_Fix;
				
				this->BoundIsSet = true;
				return true;
			}

			void printAngle()
			{
				std::cerr << "segmentation_Left_Bound: " << this->segmentation_Left_Bound << std::endl;
				std::cerr << "segmentation_Right_Bound: " << this->segmentation_Right_Bound << std::endl;
				std::cerr << "segmentation_Up_Bound: " << this->segmentation_Up_Bound << std::endl;
				std::cerr << "segmentation_Down_Bound: " << this->segmentation_Down_Bound << std::endl;
			}

			typename pcl::PointCloud<PointT>::Ptr division(const typename pcl::PointCloud<PointT>::Ptr &input, const bool &keep_Inside = true)
			{
				if(!BoundIsSet)
				{
					std::cerr << "\nBound parameter is not set\n";
					return input;
				}

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;
				
				int division_num = std::ceil(input->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
				
				cloud->points = divisionPart(division_num, input->points.begin(), input->points.end());

				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
					std::cerr << "Left_Bound: " << this->Left_Bound << std::endl;
					std::cerr << "Right_Bound: " << this->Right_Bound << std::endl;
					std::cerr << "Up_Bound: " << this->Up_Bound << std::endl;
					std::cerr << "Down_Bound: " << this->Down_Bound << std::endl;
				return cloud;
			}
		private:
			bool pointIsInside(const PointT &point)
			{
				if(std::fabs(point.x*point.y*point.z) == 0.0){ return false; }

				double radius = myFunction::norm(point.x, point.y, point.z);

				double X_Pojection = (point.x * this->camera_Depth) / point.z;
				double Y_Pojection = (point.y * this->camera_Depth) / point.z;

				this->Left_Bound = std::fmin(this->Left_Bound, X_Pojection);
				this->Right_Bound = std::fmax(this->Right_Bound, X_Pojection);
				this->Up_Bound = std::fmin(this->Up_Bound, Y_Pojection);
				this->Down_Bound = std::fmax(this->Down_Bound, Y_Pojection);

				if(X_Pojection < this->segmentation_Left_Bound) { return false; }
				if(X_Pojection > this->segmentation_Right_Bound) { return false; }
				if(Y_Pojection < this->segmentation_Up_Bound) { return false; }
				if(Y_Pojection > this->segmentation_Down_Bound) { return false; }

				return true;
			}

			template<typename RandomIt>
			std::vector<PointT, Eigen::aligned_allocator<PointT>> divisionPart(const int &division_num, const RandomIt &beg, const RandomIt &end)
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
				auto handle = std::async(std::launch::async, &objectSegmentation2::divisionPart<RandomIt>, this, division_num, beg, mid);
				auto out(divisionPart<RandomIt>(division_num, mid, end));
				auto out1(handle.get());
				
				std::copy(out1.begin(),  out1.end(), std::back_inserter(out));

				return out;
			}
	};
}

#endif