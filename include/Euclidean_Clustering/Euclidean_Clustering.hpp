#ifndef _EUCLIDEAN_CLUSTERING_H_
#define _EUCLIDEAN_CLUSTERING_H_

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
#include "../../include/Clustering_Visualization/Clustering_Visualization.hpp"

namespace Clustering_Simulation
{
    class Euclidean_Clustering : public Clustering_Visualization
    {

    public:
        Euclidean_Clustering();

        ~Euclidean_Clustering();

        std::string m_cloud_name;

        std::shared_ptr<K_Means> m_k_means;

        std::shared_ptr<Clustering_Visualization> m_clustering_visualization;

        void executeEcludianClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const double t_tolarance);
    };
}

#endif // _EUCLIDEAN_CLUSTERING_H_