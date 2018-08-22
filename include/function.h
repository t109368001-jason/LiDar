#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <iostream>
#include <cmath>
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

	//std::thread::hardware_concurrency();
    template<class T>
    T getMaxPart (T input)
    {
        pcl::console::TicToc tt;
        pcl::console::TicToc tt1;
        std::cerr << "\nSegmentation...\n", tt1.tic();
		
        T cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *cloud = *input;
		std::vector<T> clouds;
        std::cerr << "\tComputing min distance...", tt.tic();
        double min_Distance = cloud->points[0].getVector3fMap().norm();
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            for(size_t j = 0; j < cloud->points.size(); j++)
            {
                if(i == j)continue;
                min_Distance = fmin((cloud->points[i].getVector3fMap() - cloud->points[j].getVector3fMap()).norm(), min_Distance);
            }
        }
        std::cerr << " >> Done: " << tt.toc() << " ms\n";

        std::cerr << "\tComputing...\n", tt.tic();
        double threshold = min_Distance * 1.1;
        T temp(new pcl::PointCloud<pcl::PointXYZ>);
        clouds.push_back(temp);
        clouds[0]->points.push_back(cloud->points[0]);
        cloud->points.erase(cloud->points.begin() + 0);
        for(int64_t h = 0; h < clouds.size(); h++)
        {
            for(int64_t i = 0; i < clouds[h]->points.size(); i++)
            {
                for(int64_t j = 0; j < cloud->points.size(); j++)
                {
                    if((clouds[h]->points[i].getVector3fMap() - cloud->points[j].getVector3fMap()).norm() < threshold)
                    {
                        clouds[h]->points.push_back(cloud->points[j]);
                        cloud->points.erase(cloud->points.begin() + j);
                    }
                }
            }
            if(cloud->points.size() > 0)
            {
                T empty(new pcl::PointCloud<pcl::PointXYZ>);
                clouds.push_back(empty);
                clouds[h+1]->points.push_back(cloud->points[0]);
                cloud->points.erase(cloud->points.begin() + 0);
            }
        }

        while(clouds.size() > 2)
        {
            threshold  = threshold * 1.1;
            for(int64_t i = 0; i < clouds.size(); i++)
            {
                for(int64_t j = 0; j < clouds[i]->points.size(); j++)
                {
                    Eigen::Vector3f current = clouds[i]->points[j].getVector3fMap();
                    for(int64_t k = i+1; k < clouds.size(); k++)
                    {
                        for(int64_t l = 0; l < clouds[k]->points.size(); l++)
                        {
                            if((current - clouds[k]->points[l].getVector3fMap()).norm() < threshold)
                            {
                                clouds[i]->points.push_back(clouds[k]->points[l]);
                                clouds[k]->points.erase(clouds[k]->points.begin() + l);
                                l--;
                            }
                        }
                        if(clouds[k]->points.size() == 0)
                        {
                            clouds.erase(clouds.begin() + k);
                            k--;
                        }
                    }
                }
            }
		}

        std::cerr << "\t >> Done: " << tt.toc() << " ms\n\n";
        std::cerr << " >> Done: " << tt1.toc() << " ms\n\n";
        if(clouds[0]->points.size() > clouds[1]->points.size())
        {
            return clouds[0];
        }
        return clouds[1];
    }
	void pcd_to_poissonMesh(std::string filename, pcl::PolygonMesh &poission);
	pcl::PolygonMesh pcd_to_poissonMesh(std::string filename);

	PointXYZR pcd_to_XYZR(std::string filename)
	{
		PointXYZR test;
		test.set(filename);

		return test;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_to_XYZ(std::string filename)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(filename, cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_to_XYZI(std::string filename)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PCLPointCloud2 cloud_blob;
		pcl::io::loadPCDFile(filename, cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);

		return cloud;
	}
	template<class T>
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZ_to_RGB(T cloud_in)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
		double min_Distance = cloud_in->points[0].getVector3fMap().norm();
		double max_Distance = 0.0;

		for(size_t i = 0; i < cloud_in->points.size(); i++)
		{
			max_Distance = fmax((double)(cloud_in->points[i].getVector3fMap().norm()), max_Distance);
			min_Distance = fmin((double)(cloud_in->points[i].getVector3fMap().norm()), min_Distance);
		}
		for(size_t i = 0; i < cloud_in->points.size(); i++)
		{
			pcl::PointXYZRGB point;
			point.x = cloud_in->points[i].x;
			point.y = cloud_in->points[i].y;
			point.z = cloud_in->points[i].z;
			uint8_t r;
			r = ((point.getVector3fMap().norm() - min_Distance) * 255.0 / (max_Distance - min_Distance));
			r = 255 - r;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
				static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			cloud_out->points.push_back (point);
		}
		return cloud_out;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr XYZR_to_XYZRGB(PointXYZR xyzr, int fix_Num = 0, int points_of_fix = 9)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz = xyzr.cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		xyzr.max_Distance = 0.0;
		xyzr.min_Distance = xyz->points[0].getVector3fMap().norm();

		for(size_t i = 0; i < xyz->points.size(); i++)
		{
			xyzr.max_Distance = fmax((double)(xyz->points[i].getVector3fMap().norm()), xyzr.max_Distance);
			xyzr.min_Distance = fmin((double)(xyz->points[i].getVector3fMap().norm()), xyzr.min_Distance);
		}

		for(size_t i = 0; i < xyz->points.size(); i++)
		{
			pcl::PointXYZRGB point;
			point.x = xyz->points[i].x;
			point.y = xyz->points[i].y;
			point.z = xyz->points[i].z;
			uint8_t r;
			r = ((point.getVector3fMap().norm() - xyzr.min_Distance) * 255.0 / (xyzr.max_Distance - xyzr.min_Distance));
			r = 255 - r;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
				static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			xyzrgb->points.push_back (point);
		}
		if((fix_Num == 0)||(points_of_fix == 0))
		{
			xyzrgb->width = (int) xyzrgb->points.size();
			xyzrgb->height = 1;

			return xyzrgb;
		}

		points_of_fix += 1;
		std::vector<pcl::PointXYZRGB> y;
		/*
		double averageDistance = 0;
		double averageDistanceCount = 0;
		for(size_t h = 0; h < xyzrgb->points.size(); h++)
		{
			for(size_t i = 0; i < xyzrgb->points.size(); i++)
			{
				averageDistance += fabs((xyzrgb->points[h].getVector3fMap() - xyzrgb->points[i].getVector3fMap()).norm());
				averageDistanceCount += 1;
			}
		}
		averageDistance = averageDistance / averageDistanceCount / 2.0;
		cout << "Average distance" << averageDistance << endl;
	*/
		for(size_t h = 0; h < xyzrgb->points.size(); h++)
		{
			std::vector<pcl::PointXYZRGB> x;
			Eigen::Vector3f currentPoint = xyzrgb->points[h].getVector3fMap();
			for(size_t i = 0; i < xyzrgb->points.size(); i++)
			{
				if(xyzr.ring.size() > 0)
				{
					if(xyzr.ring[h] == xyzr.ring[i]) continue;
				}
				//if((xyzrgb->points[h].z/xyzrgb->points[h].getVector3fMap().norm()) == (xyzrgb->points[i].z/xyzrgb->points[i].getVector3fMap().norm()))continue;
				//if(fabs((xyzrgb->points[i].getVector3fMap() - currentPoint).norm()) < averageDistance)continue;
				if(i == h) continue;
				pcl::PointXYZRGB point = xyzrgb->points[i];
				if(x.size() == 0)
				{
					x.push_back(point);
					continue;
				}
				for(size_t j = 0; j < x.size(); j++)
				{
					if(fabs((point.getVector3fMap() - currentPoint).norm()) < fabs((x[j].getVector3fMap() - currentPoint).norm()))
					{
						x.push_back(point);
						if(x.size() > fix_Num)
						{
							x.erase(x.begin() + j);
						}
						break;
					}
				}
			}
			if(h%(xyzrgb->points.size() / 100) == 0)cout << "Calculting " << double(h) / double(xyzrgb->points.size()) *100.0 << "\%" << endl;
			for(size_t j = 0; j < (x.size()-1); j++)
			{
				pcl::PointXYZRGB point;
				Eigen::Vector3f temp = (x[j].getVector3fMap() - xyzrgb->points[h].getVector3fMap()) / points_of_fix;
				for(size_t k=1; k < points_of_fix; k++)
				{
					point.getVector3fMap() = xyzrgb->points[h].getVector3fMap() + temp * k;
			uint8_t r;
			r = ((point.getVector3fMap().norm() - xyzr.min_Distance) * 255.0 / (xyzr.max_Distance - xyzr.min_Distance));
			r = 255 - r;
					uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
						static_cast<uint32_t>(r) << 8 | static_cast<uint32_t>(r));
					point.rgb = *reinterpret_cast<float*>(&rgb);
					y.push_back (point);
				}
			}
		}

		for(size_t i = 0; i < y.size(); i++)
		{
			xyzrgb->points.push_back (y[i]);
		}

		xyzrgb->width = (int) xyzrgb->points.size();
		xyzrgb->height = 1;

		return xyzrgb;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr XYZRGB_to_XYZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*xyzrgb,*xyz);

		return xyz;
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