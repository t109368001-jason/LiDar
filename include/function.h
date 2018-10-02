#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <iostream>
#include <future>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include "../include/date.h"

using namespace std;

namespace myFunction
{

#pragma region basic

	double distance(double ax, double ay, double az, double bx = 0, double by = 0, double bz = 0)
	{
		return std::sqrt((ax-bx)*(ax-bx)+(ay-by)*(ay-by)+(az-bz)*(az-bz));
	}

	template<typename PointT>
	double distance(PointT p1, PointT p2 = PointT(0,0,0))
	{
		return distance(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
	}
	template<typename PointT>
	double norm(PointT p1)
	{
		return std::sqrt(p1.x*p1.x+p1.y*p1.y+p1.z*p1.z);
	}

	double norm(double x, double y, double z)
	{
		return std::sqrt(x*x+y*y+z*z);
	}

	void XYZ_to_Sphere(const double x, const double y, const double z, double &radius, double &phi, double &theta)
	{
		radius = norm(x,y,z);
		theta = acos(z / radius);
		phi = atan(y / x);
		
		if (x < 0.0)
		{
			if(y < 0.0)
			{
				phi -= M_PI;
			}
			else
			{
				phi += M_PI;
			}
		}
	}

	double getPhi(const double x, const double y)
	{
		double phi = atan(y / x);
		
		if (x < 0.0)
		{
			if(y < 0.0)
			{
				phi -= M_PI;
			}
			else
			{
				phi += M_PI;
			}
		}
		return phi;
	}
	
	double getTheta(const double x, const double y, const double z)
	{
		double radius = norm(x,y,z);
		double theta = acos(z / radius);
		
		return theta;
	}

	double rotateX(double &x, double &y, double &z, const double angle)
	{
		double x_ = x;
		double y_ = y;
		double z_ = z;

		x = x_;
		y = y_*std::cos(angle) + z_*std::sin(angle);
		z = y_*(-std::sin(angle)) + z_*std::cos(angle);
	}

	template<typename PointT>
	double rotateX(PointT &point, const double angle)
	{
		double x_ = point.x;
		double y_ = point.y;
		double z_ = point.z;

		point.x = x_;
		point.y = y_*std::cos(angle) + z_*std::sin(angle);
		point.z = y_*(-std::sin(angle)) + z_*std::cos(angle);
	}

	double rotateY(double &x, double &y, double &z, const double angle)
	{
		double x_ = x;
		double y_ = y;
		double z_ = z;

		x = x_*std::cos(angle) + z_*(-std::sin(angle));
		y = y_;
		z = x_*std::sin(angle) + z_*std::cos(angle);
	}

	template<typename PointT>
	double rotateY(PointT &point, const double angle)
	{
		double x_ = point.x;
		double y_ = point.y;
		double z_ = point.z;

		point.x = x_*std::cos(angle) + z_*(-std::sin(angle));
		point.y = y_;
		point.z = x_*std::sin(angle) + z_*std::cos(angle);
	}

	double rotateZ(double &x, double &y, double &z, const double &angle)
	{
		double x_ = x;
		double y_ = y;
		double z_ = z;

		z = z_;
		x = x_*std::cos(angle) + y_*std::sin(angle);
		y = x_*(-std::sin(angle)) + y_*std::cos(angle);
	}

	template<typename PointT>
	double rotateZ(PointT &point, const double &angle)
	{
		double x_ = point.x;
		double y_ = point.y;
		double z_ = point.z;

		point.z = z_;
		point.x = x_*std::cos(angle) + y_*std::sin(angle);
		point.y = x_*(-std::sin(angle)) + y_*std::cos(angle);
	}

	template<typename Type>
	std::string commaFix(Type input)
	{
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << std::fixed << input;
        
		return ss.str();
	}

	bool fileExists(std::string &filename)
	{
		struct stat buffer;
		return stat(filename.c_str(), &buffer) == 0;
	}

	pcl::PolygonMesh stl_to_mesh(std::string filename)
	{
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFileSTL(filename, mesh);

		return mesh;
	}

#pragma endregion basic
	
#pragma region value_to_RGB

	void value_to_RGB(const double value, const double min, const double max, uint8_t &r, uint8_t &g, uint8_t &b)
	{
		double temp = (value - min)/(max-min);
		if(temp > 1.0) temp = 1.0;
		else if(temp < 0.0) temp = 0.0;

		temp *= 1023;

		if(temp < 128.0)
		{
			r = 0;
			g = 0;
			b = 128 + temp;
		}
		else if(temp < (384))
		{
			r = 0;
			g = temp - 128;
			b = 255;
		}
		else if(temp < (640))
		{
			r = (temp - 384);
			g = 255;
			b = 255 - (temp - 384);
		}
		else if(temp < (896))
		{
			r = 255;
			g = 255 - (temp - 640);
			b = 0;
		}
		else if(temp < (1024))
		{
			r = 255 - (temp - 896);
			g = 0;
			b = 0;
		}
		else
		{
			r = 127;
			g = 0;
			b = 0;
		}
	}

#pragma endregion value_to_RGB

#pragma region getNearestPointsDistance

	//取得點雲中最近的兩個點距離
	template<typename RandomIt, typename PointT>
	double getNearestPointsDistancePart(int division_num, typename pcl::search::KdTree<PointT>::Ptr tree, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			double sqr_out = std::numeric_limits<double>::max();
			for(auto it = beg; it != end; ++it)
			{
				std::vector<int> indices (2);
				std::vector<float> sqr_distances (2);

				tree->nearestKSearch (*it, 2, indices, sqr_distances);

				if ((sqr_distances[1] < sqr_out)&&(sqr_distances[1] != 0.0)) sqr_out = sqr_distances[1];
			}
			return std::sqrt(sqr_out);
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, getNearestPointsDistancePart<RandomIt, PointT>, division_num, tree, beg, mid);
		auto sqr_out = getNearestPointsDistancePart<RandomIt, PointT>(division_num, tree, mid, end);
		auto sqr_out1 = handle.get();

		if(sqr_out1 < sqr_out) sqr_out = sqr_out1;

		return std::sqrt(sqr_out);
	}

	//取得點雲中最近的兩個點距離取得點雲中最近的兩個點距離
	template<typename PointT>
	double getNearestPointsDistance(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		int division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
 		
		tree->setInputCloud(cloud);
		
		return getNearestPointsDistancePart<decltype(cloud->points.begin()), PointT>(division_num, tree, cloud->points.begin(), cloud->points.end());
	}

#pragma endregion getNearestPointsDistance

#pragma region getFarthestPointsDistance
	//取得點雲中最遠的兩個點距離
	template<typename RandomIt>
	double getFarthestPointsDistancePart(int division_num, RandomIt beg1, RandomIt end1, RandomIt beg2, RandomIt end2)
	{
		auto len1 = end1 - beg1;

		if(len1 < division_num)
		{
			double out = -std::numeric_limits<double>::max();
			for(auto it1 = beg1; it1 != end1; ++it1)
			{
				for(auto it2 = beg2; it2 != end2; ++it2)
				{
					if(it1 == it2) continue;
					double dist = distance(*it1, *it2);
					if(dist > out) out = dist;
				}
			}
			return out;
		}
		auto mid1 = beg1 + len1/2;
		auto handle = std::async(std::launch::async, getFarthestPointsDistancePart<RandomIt>, division_num, beg1, mid1, beg2, end2);
		auto out = getFarthestPointsDistancePart<RandomIt>(division_num, mid1, end1, beg2, end2);
		auto out1 = handle.get();

		if(out1 < out) out = out1;

		return std::sqrt(out);
	}
	
	//取得點雲中最遠的兩個點距離
	template<typename PointT>
	double getFarthestPointsDistance(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		int division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		return getFarthestPointsDistancePart<decltype(cloud->points.begin())>(division_num, cloud->points.begin(), cloud->points.end(), cloud->points.begin(), cloud->points.end());
	}

#pragma endregion getFarthestPointsDistance

#pragma region getNearOrFarthestPoint

	//取得點雲中距離point最遠的點
	template<typename RandomIt, typename PointT>
	PointT getNearOrFarthestPointPart(int division_num, bool Nearest, PointT point, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			double current;
			PointT out;
			for(auto it = beg; it != end; ++it)
			{
				double temp = distance(point.x, point.y, point.z, (*it).x, (*it).y, (*it).z);
				if((temp > current) ^ Nearest)
				{
					current = temp;
					out = *it;
				}
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, getNearOrFarthestPointPart<RandomIt, PointT>, division_num, Nearest, point, beg, mid);
		auto out = getNearOrFarthestPointPart<RandomIt, PointT>(division_num, Nearest, point, mid, end);
		auto out1 = handle.get();

		if((distance(point.x, point.y, point.z, out1.x, out1.y, out1.z) > distance(point.x, point.y, point.z, out.x, out.y, out.z)) ^ Nearest) return out1;

		return out;
	}

	template<typename PointT>
	PointT getNearOrFarthestPoint(typename pcl::PointCloud<PointT>::Ptr cloud, bool Nearest = true, PointT point = PointT(0,0,0))
	{
		int division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		return getNearOrFarthestPointPart<decltype(cloud->points.begin()), PointT>(division_num, Nearest, point, cloud->points.begin(), cloud->points.end());
	}

#pragma endregion getNearOrFarthestPoint

#pragma region pcd_to_poissonMesh

	void pcd_to_poissonMesh(std::string filename, pcl::PolygonMesh &poission)
	{
		string ply_filename = filename.substr(0,filename.find_last_of('.'))+"_poission.ply";

		if (pcl::io::loadPLYFile(ply_filename, poission) == -1)
		{
			pcl::PCDReader reader;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			reader.read (filename, *cloud);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloud);
			n.setInputCloud(cloud);
			n.setSearchMethod(tree);
			n.setKSearch(30);
			n.compute(*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud(cloud_with_normals);

			pcl::Poisson<pcl::PointNormal> poisson;
			poisson.setDepth(8);
			poisson.setSolverDivide(8);
			poisson.setIsoDivide(8);
			poisson.setPointWeight(4.0f);
			poisson.setInputCloud(cloud_with_normals);

			poisson.reconstruct(poission);
			pcl::io::savePLYFile(ply_filename, poission);
		}
	}

	pcl::PolygonMesh pcd_to_poissonMesh(std::string filename)
	{
		string ply_filename = filename.substr(0,filename.find_last_of('.'))+"_poission.ply";
		pcl::PolygonMesh poission;
		if (pcl::io::loadPLYFile(ply_filename, poission) == -1)
		{
			pcl::PCDReader reader;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

			reader.read (filename, *cloud);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud(cloud);
			n.setInputCloud(cloud);
			n.setSearchMethod(tree);
			n.setKSearch(30);
			n.compute(*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud(cloud_with_normals);

			pcl::Poisson<pcl::PointNormal> poisson;
			poisson.setDepth(8);
			poisson.setSolverDivide(8);
			poisson.setIsoDivide(8);
			poisson.setPointWeight(4.0f);
			poisson.setInputCloud(cloud_with_normals);

			poisson.reconstruct(poission);
			pcl::io::savePLYFile(ply_filename, poission);
		}
		return poission;
	}
	
#pragma endregion pcd_to_poissonMesh

#pragma region loadMultiPCD

	template<typename RandomIt1, typename RandomIt2, typename PointT>
	int loadMultiPCDPart(int division_num, RandomIt1 beg1, RandomIt1 end1, RandomIt2 beg2, RandomIt2 end2)
	{
		auto len1 = end1 - beg1;
		auto len2 = end2 - beg2;

		if(len1 < division_num)
		{
			int out = 0;
			auto it2 = beg2;
			for(auto it1 = beg1; it1 != end1; ++it1,++it2)
			{
				typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
				*it2 = temp;
				pcl::io::loadPCDFile(*it1, **it2);
				out++;
			}
			return out;
		}

		auto mid1 = beg1 + len1/2;
		auto mid2 = beg2 + len1/2;
		auto handle = std::async(std::launch::async, loadMultiPCDPart<RandomIt1, RandomIt2, PointT>, division_num, beg1, mid1, beg2, mid2);
		auto out1 = loadMultiPCDPart<RandomIt1, RandomIt2, PointT>(division_num, mid1, end1, mid2, end2);
		auto out = handle.get();

		return out + out1;
	}

	template<typename PointT>
	int loadMultiPCD(std::string filename, std::vector<boost::shared_ptr<typename pcl::PointCloud<PointT>>> &clouds)
	{
		std::ifstream fs;
		std::string line;
		std::vector<std::string> lines;

		fs.open(filename);

		while(!fs.eof())
		{
			std::getline(fs, line);

			boost::trim(line);
			if(line == "") continue;
			lines.push_back(line);
		}

		clouds.resize(lines.size());

		int division_num = std::ceil(lines.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		int num = loadMultiPCDPart<decltype(lines.begin()), decltype(clouds.begin()), PointT>(division_num, lines.begin(), lines.end(), clouds.begin(), clouds.end());
		
		return num;
	}

#pragma endregion loadMultiPCD

#pragma region XYZ_to_XYZRGB

	template<typename RandomIt>
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> XYZ_to_XYZRGBPart(int division_num, double min_Distance, double div, bool gray, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> out;
			uint32_t rgb;
			pcl::PointXYZRGB point;
			for(auto it = beg; it != end; ++it)
			{
				point.x = (*it).x;
				point.y = (*it).y;
				point.z = (*it).z;
				if(gray)
				{
					uint8_t r;
					r = ((std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z) - min_Distance) * 255.0 / div);
					r = 255 - r;
					rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
				}
				else
				{
					uint8_t r;
					uint8_t g;
					uint8_t b;
					value_to_RGB(std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z), min_Distance, div + min_Distance, r, g, b);
					rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				}
				
				point.rgb = *reinterpret_cast<float*>(&rgb);
				out.push_back (point);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, XYZ_to_XYZRGBPart<RandomIt>, division_num, min_Distance, div, gray, beg, mid);
		auto out = XYZ_to_XYZRGBPart(division_num, min_Distance, div, gray, mid, end);
		auto out1 = handle.get();

		std::copy(out1.begin(), out1.end(), std::back_inserter(out));

		return out;
	}

	template<typename PointT>
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZ_to_XYZRGB(typename pcl::PointCloud<PointT>::Ptr cloud_in, bool gray = true)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
		double min_Distance = distance<PointT>(getNearOrFarthestPoint<PointT>(cloud_in));
		double max_Distance = distance<PointT>(getNearOrFarthestPoint<PointT>(cloud_in, false));
		double div = max_Distance - min_Distance;
		int division_num = std::ceil(cloud_in->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		cloud_out->points = XYZ_to_XYZRGBPart(division_num, min_Distance, div, gray, cloud_in->points.begin(), cloud_in->points.end());
		cloud_out->width = (int) cloud_out->points.size();
		cloud_out->height = 1;

		return cloud_out;
	}

#pragma endregion XYZ_to_XYZRGB

#pragma region fillColor

	template<typename RandomIt>
	int fillColorPart(int division_num, uint8_t r, uint8_t g, uint8_t b, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			int out = 0;
			uint32_t rgb;
			for(auto it = beg; it != end; ++it)
			{
				rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				(*it).rgb = *reinterpret_cast<float*>(&rgb);
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, fillColorPart<RandomIt>, division_num, r, g, b, beg, mid);
		auto out1 = fillColorPart<RandomIt>(division_num, r, g, b, mid, end);
		auto out = handle.get();

		return out + out1;
	}

	int fillColor(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, uint8_t r, uint8_t g, uint8_t b)
	{
		int division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		return fillColorPart<decltype(cloud->points.begin())>(division_num, r, g, b, cloud->points.begin(), cloud->points.end());
	}

	template<typename PointT>
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fillColor(typename pcl::PointCloud<PointT>::Ptr cloud, uint8_t r, uint8_t g, uint8_t b)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointXYZRGB point;
		uint32_t rgb;

		for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it)
		{
			point.x = (*it).x;
			point.y = (*it).y;
			point.z = (*it).z;
			rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			out->points.push_back (point);
		}
		
		out->width = (int) out->points.size();
		out->height = 1;

		return out;
	}

#pragma endregion fillColor

#pragma region getOrigin

	template<typename RandomIt, typename PointT>
	PointT getOriginPart(int division_num, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			PointT out;
			out.x = 0;
			out.y = 0;
			out.z = 0;
			for(auto it = beg; it != end; ++it)
			{
				out.x += (*it).x;
				out.y += (*it).y;
				out.z += (*it).z;
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, getOriginPart<RandomIt, PointT>, division_num, beg, mid);
		auto out = getOriginPart<RandomIt, PointT>(division_num, mid, end);
		auto out1 = handle.get();

		out.x += out1.x;
		out.y += out1.y;
		out.z += out1.z;

		return out;
	}
	template<typename PointT>
	PointT getOrigin(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		int division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		PointT out = getOriginPart<decltype(cloud->points.begin()), PointT>(division_num, cloud->points.begin(), cloud->points.end());
		
		out.x = out.x / (double)cloud->points.size();
		out.y = out.y / (double)cloud->points.size();
		out.z = out.z / (double)cloud->points.size();

		return out;
	}

#pragma endregion getOrigin

#pragma region offsetToOrigin

	template<typename RandomIt, typename PointT>
	int offsetToOriginPart(int division_num, PointT point, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			int out;
			for(auto it = beg; it != end; ++it)
			{
				(*it).x -= point.x;
				(*it).y -= point.y;
				(*it).z -= point.z;
				out++;
			}
			return out;
		}
		auto mid = beg + len/2;
		auto handle = std::async(std::launch::async, offsetToOriginPart<RandomIt, PointT>, division_num, point, beg, mid);
		auto out = offsetToOriginPart<RandomIt, PointT>(division_num, point, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}
	template<typename PointT>
	int offsetToOrigin(typename pcl::PointCloud<PointT>::Ptr &cloud)
	{
		int division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		return offsetToOriginPart(division_num, getOrigin<PointT>(cloud), cloud->points.begin(), cloud->points.end());
	}

#pragma endregion offsetToOrigin
	
#pragma region points_to_pcl

	template<typename RandomIt, typename RandomIt2>
	int points_to_pclPart(int division_num, RandomIt2 ptr, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			int out;
			for(auto it = beg; it != end; ++it)
			{
				(*it).x = ptr->x;
				(*it).y = ptr->y;
				(*it).z = ptr->z;
				ptr++;
				out++;
			}
			return out;
		}
		auto mid = beg + len/2;
		auto ptr2 = ptr + len/2;
		auto handle = std::async(std::launch::async, points_to_pclPart<RandomIt, RandomIt2>, division_num, ptr, beg, mid);
		auto out = points_to_pclPart<RandomIt, RandomIt2>(division_num, ptr2, mid, end);
		auto out1 = handle.get();

		return out + out1;
	}

	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr points_to_pcl(const rs2::points& points)
	{
		int division_num = std::ceil(points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

		auto sp = points.get_profile().as<rs2::video_stream_profile>();
		cloud->width = sp.width();
		cloud->height = sp.height();
		cloud->is_dense = false;
		cloud->points.resize(points.size());
		auto ptr = points.get_vertices();

		int count = points_to_pclPart(division_num, ptr, cloud->points.begin(), cloud->points.end());
		/*for (auto& p : cloud->points)
		{
			p.x = ptr->x;
			p.y = ptr->y;
			p.z = ptr->z;
			ptr++;
		}*/

		return cloud;
	}

#pragma endregion points_to_pcl

#pragma region getChanges

	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr getChanges(typename pcl::PointCloud<PointT>::Ptr cloud1, typename pcl::PointCloud<PointT>::Ptr cloud2, double resolution)
	{
		typename pcl::PointCloud<PointT>::Ptr temp(new typename pcl::PointCloud<PointT>);
		pcl::octree::OctreePointCloudChangeDetector<PointT> octree (resolution);
		std::vector<int> newPointIdxVector;

		octree.setInputCloud(cloud1);
		octree.addPointsFromInputCloud();
		octree.switchBuffers ();
		octree.setInputCloud (cloud2);
		octree.addPointsFromInputCloud ();
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);

		for(auto it = newPointIdxVector.begin(); it != newPointIdxVector.end(); ++it)
		{
			temp->points.push_back(cloud2->points[*it]);
		}

		temp->width = (int) temp->points.size();
		temp->height = 1;

		return temp;
	}

#pragma endregion getChanges

#pragma region printCamera

	void printCamera(pcl::visualization::Camera camera)
	{
		std::cout << "Cam: " << endl;
        std::cout << " - pos: (" << camera.pos[0] << ", "    << camera.pos[1] << ", "    << camera.pos[2] << ")" << endl;
        std::cout << " - view: ("    << camera.view[0] << ", "   << camera.view[1] << ", "   << camera.view[2] << ")"    << endl;
        std::cout << " - focal: ("   << camera.focal[0] << ", "  << camera.focal[1] << ", "  << camera.focal[2] << ")"   << endl;
	}

#pragma endregion printCamera

#pragma region showCloud

	void showCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, name);
	}

#pragma endregion showCloud

#pragma region updateCloud

	void updateCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name)
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

		if( !viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, name);
		}
	}

#pragma endregion updateCloud

#pragma region millisecondToString

	std::string millisecondToString(std::chrono::milliseconds &duration, bool isFileName = true)
	{
		std::ostringstream stream;
		std::chrono::time_point<std::chrono::system_clock> tp = std::chrono::time_point<std::chrono::system_clock>(duration);
		tp += std::chrono::hours(8);
		auto dp = date::floor<date::days>(tp);  // dp is a sys_days, which is a
										// type alias for a C::time_point
		auto date = date::year_month_day{dp};
		auto time = date::make_time(std::chrono::duration_cast<std::chrono::milliseconds>(tp-dp));
		stream << std::setfill('0') << std::setw(4) << date.year().operator int();
		if(!isFileName) stream << '/';
		stream << std::setfill('0') << std::setw(2) << date.month().operator unsigned int();
		if(!isFileName) stream << '/';
		stream << std::setfill('0') << std::setw(2) << date.day().operator unsigned int();
		if(!isFileName) stream << ' ';
		else stream << '_';
		stream << std::setfill('0') << std::setw(2) << time.hours().count();
		if(!isFileName) stream << ':';
		stream << std::setfill('0') << std::setw(2) << time.minutes().count();
		if(!isFileName) stream << ':';
		stream << std::setfill('0') << std::setw(2) << time.seconds().count();
		if(!isFileName) stream << '.';
		else stream << '_';
		stream << std::setfill('0') << std::setw(3) << time.subseconds().count();
		return stream.str();
	}

#pragma endregion millisecondToString

#pragma region getSimilarity

	template<typename PointT>
	double getSimilarity(typename pcl::PointCloud<PointT>::Ptr cloud1, typename pcl::PointCloud<PointT>::Ptr cloud2, double resolution)
	{
        pcl::octree::OctreePointCloudChangeDetector<PointT> octree (resolution);
        std::vector<int> newPointIdxVector;

        octree.setInputCloud(cloud1);
        octree.addPointsFromInputCloud();
        octree.switchBuffers ();
        octree.setInputCloud (cloud2);
        octree.addPointsFromInputCloud ();
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);

        return 1 - ((double)newPointIdxVector.size() / (double)cloud2->points.size());
	}

#pragma endregion getSimilarity

#pragma region bagFileNameToMilliseconds

	std::chrono::milliseconds bagFileNameToMilliseconds(std::string &bagFileName)
	{
		if(fileExists(bagFileName))
		{
			std::tm tm;
			std::istringstream ss(bagFileName.substr(bagFileName.rfind('/') + 1, bagFileName.rfind('.')-bagFileName.rfind('/')-1));
			ss >> std::get_time(&tm,"%Y%m%d_%H%M%S");
			return std::chrono::milliseconds(std::mktime(&tm)*1000);
		}
		return std::chrono::milliseconds(0);
	}

#pragma endregion bagFileNameToMilliseconds

	template<typename PointT>
	PointT vector_plane_cross_point(const PointT &v, const PointT &n, const PointT &p0)
	{
		PointT result;
		double t = (p0.x*n.x+p0.y*n.y+p0.z*n.z)/(v.x*n.x+v.y*n.y+v.z*n.z);

		result.x = v.x*t;
		result.y = v.y*t;
		result.z = v.z*t;
	}
	template<typename RandomIt1, typename RandomIt2> 
	int getDivNum(RandomIt1 total, RandomIt2 part)
	{
		return std::ceil((double)(total)/(double)(part));
	}		

}
#endif