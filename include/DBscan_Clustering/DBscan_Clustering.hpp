#ifndef _DBSCAN_CLUSTERING_HPP_
#define _DBSCAN_CLUSTERING_HPP_

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

#include "../../include/K_Means/K-Means.hpp"

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

namespace Clustering_Simulation
{
    class DBscan_Clustering : protected K_Means

    {
    public:
        DBscan_Clustering();

        ~DBscan_Clustering();

        std::shared_ptr<K_Means> m_k_means;

        void executeDBscanClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const double &t_min_points, const double &t_eps);

        void clusteredCloudVisualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud);
    };
} // namespace Lidar_Simulation

#endif // _DBSCAN_CLUSTERING_HPP_
