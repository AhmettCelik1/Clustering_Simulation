#ifndef _CLUSTERING_VISUALIZATION_
#define _CLUSTERING_VISUALIZATION_

// C++ System Headers
#include <iostream>
#include <unordered_map>
#include <vector>
#include <array>
#include <thread>
#include <memory>
#include <random>
#include <chrono>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace Clustering_Simulation

{
    class Clustering_Visualization
    {
    public:
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

        Clustering_Visualization();

        ~Clustering_Visualization();

        void callRawPointsVisualization(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points);

        void clusteringVisualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud, const std::string &t_cloud_name);

        void vectorToPclPointCloudConverter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud);
    };

};

#endif // _CLUSTERING_VISUALIZATION_
