#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>
#include <future>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/poisson.h>
#include <boost/algorithm/clamp.hpp>
#include <pcl/console/time.h>
#include "point_type.h"

using namespace std;
using namespace myClass;

namespace myFunction{
	//取得點雲中最近的兩個點距離
	template<class T>
	double getMinDistanceBetweenPoints(T cloud)
	{
		double minDistance = (cloud->points[0].getVector3fMap()-cloud->points[1].getVector3fMap()).norm();
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		tree->setInputCloud(cloud);
		for(int i = 0; i < cloud->points.size(); i++)
		{
			std::vector<int> result(2);
			std::vector<float> result_sqr_distance(2);
			tree->nearestKSearch(cloud->points[i],2,result,result_sqr_distance);
			minDistance = std::fmin(std::sqrt(result_sqr_distance[1]), minDistance);
		}

		return minDistance;
	}
	//取得點雲中最遠的兩個點距離
	template<class T>
	double getMaxDistanceBetweenPoints(T cloud)
	{
		double maxDistance = 0;
		
		for(int i = 0; i < cloud->points.size(); i++)
		{
			for(int j = 0; j < cloud->points.size(); j++)
			{
				if(i == j)continue;
				maxDistance = std::fmax((cloud->points[i].getVector3fMap() - cloud->points[j].getVector3fMap()).norm(), maxDistance);
			}
		}
		
		return maxDistance;
	}
	//取得點雲中距離point最近的點
	template<class T>
	pcl::PointXYZ getNearestPoint(T cloud, pcl::PointXYZ point)
	{
		pcl::PointXYZ point_out;
		std::vector<int> result(1);
		std::vector<float> result_sqr_distance(1);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

		tree->setInputCloud(cloud);
		tree->nearestKSearch(point,result.size(),result,result_sqr_distance);
		point_out = cloud->points[result[0]];

		return point_out;
	}
	//取得點雲中距離point最遠的點
	template<typename RandomIt>
	pcl::PointXYZ getFarthestPointPart(int division_num, pcl::PointXYZ point, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			double current;
			pcl::PointXYZ out;
			for(auto it = beg; it != end; ++it)
			{
				double temp = (point.getVector3fMap() - (*it).getVector3fMap()).norm();
				if(temp > current)
				{
					current = temp;
					out = *it;
				}
			}
			return out;
		}
		auto mid = beg + len/2;

		auto handle = std::async(std::launch::async, getFarthestPointPart<RandomIt>, division_num, point, beg, mid);

		auto out = getFarthestPointPart(division_num, point, mid, end);

		auto out1 = handle.get();

		if((point.getVector3fMap() - out1.getVector3fMap()).norm() > (point.getVector3fMap() - out.getVector3fMap()).norm())
		{
			return out1;
		}

		return out;
	}

	template<class T>
	pcl::PointXYZ getFarthestPoint(T cloud, pcl::PointXYZ point = pcl::PointXYZ(0,0,0))
	{
		double division_num = std::ceil(cloud->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		return getFarthestPointPart(division_num, point, cloud->points.begin(), cloud->points.end());
	}

	template<typename RandomIt>
	std::vector<pcl::PointXYZRGB> XYZ_to_XYZRGBPart(int division_num, double min_Distance, double div, RandomIt beg, RandomIt end)
	{
		auto len = end - beg;

		if(len < division_num)
		{
			std::vector<pcl::PointXYZRGB> out;
			for(auto it = beg; it != end; ++it)
			{
				pcl::PointXYZRGB point;
				point.x = (*it).x;
				point.y = (*it).y;
				point.z = (*it).z;
				uint8_t r;
				r = ((std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z) - min_Distance) * 255.0 / div);
				r = 255 - r;
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				out.push_back (point);
			}
			return out;
		}
		auto mid = beg + len/2;

		auto handle = std::async(std::launch::async, XYZ_to_XYZRGBPart<RandomIt>, division_num, min_Distance, div, beg, mid);

		auto out = XYZ_to_XYZRGBPart(division_num, min_Distance, div, mid, end);

		auto out1 = handle.get();

		std::copy(out1.begin(), out1.end(), std::back_inserter(out));

		return out;
	}

	template<class T>
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZ_to_XYZRGB(T cloud_in)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

		double max_Distance = getFarthestPoint(cloud_in, pcl::PointXYZ(0,0,0)).getVector3fMap().norm();
		double min_Distance = getNearestPoint(cloud_in, pcl::PointXYZ(0,0,0)).getVector3fMap().norm();

		double div = max_Distance - min_Distance;
		double division_num = std::ceil(cloud_in->points.size() / std::thread::hardware_concurrency()) + std::thread::hardware_concurrency();
		
		std::vector<pcl::PointXYZRGB> points = XYZ_to_XYZRGBPart(division_num, min_Distance, div, cloud_in->points.begin(), cloud_in->points.end());

		std::copy(points.begin(), points.end(), std::back_inserter(cloud_out->points));

		cloud_out->width = (int) cloud_out->points.size();
		cloud_out->height = 1;

		return cloud_out;
	}

	pcl::PolygonMesh stl_to_mesh(std::string filename)
	{
		pcl::PolygonMesh mesh;
		pcl::io::loadPolygonFileSTL(filename, mesh);

		return mesh;
	}

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
}
#endif