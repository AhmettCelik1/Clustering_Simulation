#include "../../include/K_Means/K-Means.hpp"

namespace Lidar_Simulation
{
    K_Means::K_Means() : m_inertia(0.0), m_points_color_range{{"r_min", 0}, {"r_max", 100}, {"g_min", 0}, {"g_max", 100}, {"b_min", 0}, {"b_max", 100}}
    {
    }

    K_Means::~K_Means()
    {
        std::cout << "K_Means destructor is called" << std::endl;
    }

    void K_Means::kMeansClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_cluster_number)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr t_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        t_cloud->width = t_lidar_points->size();
        t_cloud->height = 1;
        t_cloud->points.resize(t_cloud->width * t_cloud->height);

        for (size_t i{0}; i < t_lidar_points->size(); i++)
        {

            std::uniform_real_distribution<double> distr_r;

            std::uniform_real_distribution<double> distr_g;

            std::uniform_real_distribution<double> distr_b;

            std::random_device rd_r;

            std::random_device rd_g;

            std::random_device rd_b;

            std::mt19937 eng_r(rd_r());

            std::mt19937 eng_g(rd_g());

            std::mt19937 eng_b(rd_b());

            distr_r = std::uniform_real_distribution<double>(m_points_color_range["r_min"], m_points_color_range["r_max"]);

            distr_g = std::uniform_real_distribution<double>(m_points_color_range["g_min"], m_points_color_range["g_max"]);

            distr_b = std::uniform_real_distribution<double>(m_points_color_range["b_min"], m_points_color_range["b_max"]);

            t_cloud->points[i].x = t_lidar_points->at(i).at(0).at(0);
            t_cloud->points[i].y = t_lidar_points->at(i).at(1).at(0);
            t_cloud->points[i].z = t_lidar_points->at(i).at(2).at(0);
            t_cloud->points[i].r = 0;
            t_cloud->points[i].g = 10;
            t_cloud->points[i].b = 40;

            std::cout << "disr_r: " << distr_r(eng_r) << std::endl;
            std::cout << "disr_g: " << distr_g(eng_g) << std::endl;
            std::cout << "disr_b: " << distr_b(eng_b) << std::endl;
        }

        pcl::visualization::PCLVisualizer viewer("t_cloud viewer");

        viewer.addPointCloud(t_cloud, "t_cloud");

        viewer.setBackgroundColor(0, 0, 0);

        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "t_cloud",1);

        viewer.addCoordinateSystem(1.0);

        viewer.initCameraParameters();
    }
    cv::Mat K_Means::getKMeansMat(cv::Mat &t_mat, const size_t &t_cluster_number)
    {
        cv::Mat labels;
        cv::Mat centers;
        cv::kmeans(t_mat, t_cluster_number, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);
        return centers;
    }
}

// cv::Mat t_mat;
// t_mat.create(t_lidar_points->size(), 6, CV_32FC1);
// cv::Mat bestLabels, centers, clustered;

// for (size_t i{0}; i < t_lidar_points->size(); i++)
// {
//     t_mat.at<float>(i, 0) = t_lidar_points->at(i).at(0).at(0);
//     t_mat.at<float>(i, 1) = t_lidar_points->at(i).at(1).at(0);
//     t_mat.at<float>(i, 2) = t_lidar_points->at(i).at(2).at(0);
//     t_mat.at<float>(i, 3) =50;
//     t_mat.at<float>(i, 4) =130;
//     t_mat.at<float>(i, 5) =270;
// }

// cv::kmeans(t_mat,t_cluster_number,bestLabels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

// int colors[t_cluster_number];

// for (size_t i{0}; i < t_cluster_number; i++)
// {
//     colors[i] =255 / t_cluster_number * i;
// }

// clustered.create(t_lidar_points->size(), 3, CV_32FC1);

// for (size_t i{0}; i < t_lidar_points->size(); i++)
// {
//     clustered.at<float>(i, 0) = colors[bestLabels.at<int>(i, 0)];
//     clustered.at<float>(i, 1) = colors[bestLabels.at<int>(i, 0)];
//     clustered.at<float>(i, 2) = colors[bestLabels.at<int>(i, 0)];
// }

// clustered.convertTo(clustered,CV_8U);
// cv::imshow("Clustered", clustered);
// cv::waitKey(0);
