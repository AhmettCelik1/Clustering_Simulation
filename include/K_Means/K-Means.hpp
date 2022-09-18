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

        cv::Mat t_mat;

        double m_inertia;

            std::unordered_map<std::string, double> m_points_color_range;

        void kMeansClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_cluster_number);

        cv::Mat getKMeansMat(cv::Mat &t_mat, const size_t &t_cluster_number);
    };
} // Namespace  Lidar_Simulation

#endif // _K_MEANS_H_