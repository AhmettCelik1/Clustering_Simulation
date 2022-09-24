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

#include "../../include/Clustering_Visualization/Clustering_Visualization.hpp"

namespace Clustering_Simulation
{
    class K_Means : public Clustering_Visualization
    {

    public:
        K_Means();

        ~K_Means();

        std::shared_ptr<std::vector<std::vector<std::vector<double>>>> m_lidar_points;

        size_t m_cluster_number;

        std::string m_kmeans_name;


        std::shared_ptr<Clustering_Visualization> m_clustering_visualization;

        void kMeansClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_cluster_number);
    };
} // Namespace  Lidar_Simulation

#endif // _K_MEANS_H_