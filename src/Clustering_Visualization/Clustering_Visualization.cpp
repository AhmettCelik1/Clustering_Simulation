#include "../../include/Clustering_Visualization/Clustering_Visualization.hpp"

namespace Clustering_Simulation
{

    Clustering_Visualization::Clustering_Visualization() : m_points_color_range{{"r_min", 10},
                                                                                {"r_max", 255},
                                                                                {"g_min", 10},
                                                                                {"g_max", 255},
                                                                                {"b_min", 10},
                                                                                {"b_max", 255}},
                                                           eng_r(rd_r()), eng_g(rd_g()), eng_b(rd_b())
    {
        distr_r = std::uniform_real_distribution<double>(m_points_color_range["r_min"], m_points_color_range["r_max"]);

        distr_g = std::uniform_real_distribution<double>(m_points_color_range["g_min"], m_points_color_range["g_max"]);

        distr_b = std::uniform_real_distribution<double>(m_points_color_range["b_min"], m_points_color_range["b_max"]);
    }

    Clustering_Visualization::~Clustering_Visualization()
    {
    }

    void Clustering_Visualization::callRawPointsVisualization(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        vectorToPclPointCloudConverter(t_lidar_points, cloud);

        std::string cloud_name = "Raw Points";

        clusteringVisualization(cloud, cloud_name);
    }

    void Clustering_Visualization::vectorToPclPointCloudConverter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud)
    {
        t_cloud->width = t_lidar_points->size();
        t_cloud->height = 1;
        t_cloud->points.resize(t_cloud->width * t_cloud->height);

        for (size_t i = 0; i < t_lidar_points->size(); i++)
        {
            t_cloud->points[i].x = t_lidar_points->at(i).at(0).at(0);
            t_cloud->points[i].y = t_lidar_points->at(i).at(1).at(0);
            t_cloud->points[i].z = t_lidar_points->at(i).at(2).at(0);
            t_cloud->points[i].r = distr_r(eng_r);
            t_cloud->points[i].g = distr_g(eng_g);
            t_cloud->points[i].b = distr_b(eng_b);
        }
    }

    void Clustering_Visualization::clusteringVisualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud, const std::string &t_cloud_name)
    {
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << " In order to go back menu click on the viewer window and press '❌' " << std::endl;

        pcl::visualization::PCLVisualizer viewer(t_cloud_name);

        viewer.addPointCloud(t_cloud);

        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

        viewer.setBackgroundColor(0.0, 0.0, 0.0);

        viewer.addCoordinateSystem(1, 0, 0, 0);

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

}