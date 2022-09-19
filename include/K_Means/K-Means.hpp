#ifndef _K_MEANS_H_
#define _K_MEANS_H_

// C++ System Headers
#include <iostream>
#include <unordered_map>
#include <vector>
#include <array>
#include <thread>
#include <memory>
#include <random>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/ml/kmeans.h"
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

namespace Lidar_Simulation
{
    class K_Means
    {

    public:
        K_Means();

        ~K_Means();

        std::unordered_map<std::string, double> m_points_color_range;

        std::uniform_real_distribution<double> distr_r;

        std::uniform_real_distribution<double> distr_g;

        std::uniform_real_distribution<double> distr_b;

        std::random_device rd_r;

        std::random_device rd_g;

        std::random_device rd_b;

        std::mt19937 eng_r;

        std::mt19937 eng_g;

        std::mt19937 eng_b;

        std::shared_ptr<std::vector<std::vector<std::vector<double>>>> m_lidar_points;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud_cluster;

        size_t m_cluster_number;

        vtkRenderWindowInteractor *m_interactor;

        std::thread m_cluster_thread;

        std::thread m_raw_thread;

        bool m_flag;

        void kMeansClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_cluster_number);

        void rawpointCloudVisualizationThread();

        void clusteredCloudVisualizationThread();

        void callRawData(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points);

        void pclPointCloudConverter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud);
    };
} // Namespace  Lidar_Simulation

#endif // _K_MEANS_H_