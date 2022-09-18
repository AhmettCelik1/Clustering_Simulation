#include "../../include/K_Means/K-Means.hpp"

namespace Lidar_Simulation
{
    K_Means::K_Means() : m_inertia(0.0), m_points_color_range{{"r_min", 0},
                                                              {"r_max", 255},
                                                              {"g_min", 0},
                                                              {"g_max", 255},
                                                              {"b_min", 0},
                                                              {"b_max", 255}},
                         m_flag{false}
    {
    }

    K_Means::~K_Means()
    {
        std::cout << "K_Means destructor is called" << std::endl;
    }

    void K_Means::kMeansClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_cluster_number)
    {

        m_flag = true;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud->width = t_lidar_points->size();

        cloud->height = 1;

        cloud->points.resize(cloud->width * cloud->height);

        std::uniform_real_distribution<double> distr_x;

        std::uniform_real_distribution<double> distr_y;

        std::uniform_real_distribution<double> distr_z;

        std::random_device rd_x;

        std::random_device rd_y;

        std::random_device rd_z;

        std::mt19937 eng_x(rd_x());

        std::mt19937 eng_y(rd_y());

        std::mt19937 eng_z(rd_z());

        distr_x = std::uniform_real_distribution<double>(m_points_color_range["r_min"], m_points_color_range["r_max"]);

        distr_y = std::uniform_real_distribution<double>(m_points_color_range["g_min"], m_points_color_range["g_max"]);

        distr_z = std::uniform_real_distribution<double>(m_points_color_range["b_min"], m_points_color_range["b_max"]);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {

            cloud->points[i].x = t_lidar_points->at(i).at(0).at(0);

            cloud->points[i].y = t_lidar_points->at(i).at(1).at(0);

            cloud->points[i].z = t_lidar_points->at(i).at(2).at(0);

            cloud->points[i].r = int(distr_x(eng_x));

            cloud->points[i].g = int(distr_y(eng_y));

            cloud->points[i].b = int(distr_z(eng_z));
        }

        m_pcl_cloud = cloud;
        m_cluster_number = t_cluster_number;

        pcl::Kmeans real(static_cast<int>(cloud->size()), 3);

        real.setClusterSize(static_cast<int>(t_cluster_number));

        for (size_t i = 0; i < cloud->size(); ++i)
        {

            std::vector<float> data(3);

            data[0] = cloud->points[i].x;

            data[1] = cloud->points[i].y;

            data[2] = cloud->points[i].z;
            real.addDataPoint(data);
        }

        real.kMeans();

        // get the cluster ceontroid

        pcl::Kmeans::Centroids centroids = real.get_centroids();

        for (size_t i{0}; i < centroids.size(); i++)
        {

            std::cout << i << "_cent output: x: " << centroids[i][0] << " ,";
            std::cout << "y: " << centroids[i][1] << " ,";
            std::cout << "z: " << centroids[i][2] << std::endl;
        }

        printf("points in total cloud: %d \t", static_cast<int>(cloud->size()));
        printf("cluster number: %d \t", static_cast<int>(t_cluster_number));
        printf("centroids size: %d \t", static_cast<int>(centroids.size()));

        // visualize the centroid cloud

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        centroid_cloud->width = centroids.size();

        centroid_cloud->height = 1;

        centroid_cloud->points.resize(centroid_cloud->width * centroid_cloud->height);

        for (size_t i = 0; i < centroid_cloud->points.size(); ++i)
        {

            centroid_cloud->points[i].x = centroids[i][0];

            centroid_cloud->points[i].y = centroids[i][1];

            centroid_cloud->points[i].z = centroids[i][2];

            centroid_cloud->points[i].r = int(distr_x(eng_x));

            centroid_cloud->points[i].g = int(distr_y(eng_y));

            centroid_cloud->points[i].b = int(distr_z(eng_z));
        }

        m_centroid_cloud = centroid_cloud;

        std::thread t1(&K_Means::clusteredCloudVisualizationThread, this, centroid_cloud, t_cluster_number);

        t1.detach();
        m_flag = false;
        std::cout << std::endl;
        std::cout << std::endl;
    }

    void K_Means::rawpointCloudVisualizationThread(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud->width = t_lidar_points->size();

        cloud->height = 1;

        cloud->points.resize(cloud->width * cloud->height);

        std::uniform_real_distribution<double> distr_x;

        std::uniform_real_distribution<double> distr_y;

        std::uniform_real_distribution<double> distr_z;

        std::random_device rd_x;

        std::random_device rd_y;

        std::random_device rd_z;

        std::mt19937 eng_x(rd_x());

        std::mt19937 eng_y(rd_y());

        std::mt19937 eng_z(rd_z());

        distr_x = std::uniform_real_distribution<double>(m_points_color_range["r_min"], m_points_color_range["r_max"]);

        distr_y = std::uniform_real_distribution<double>(m_points_color_range["g_min"], m_points_color_range["g_max"]);

        distr_z = std::uniform_real_distribution<double>(m_points_color_range["b_min"], m_points_color_range["b_max"]);

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {

            cloud->points[i].x = t_lidar_points->at(i).at(0).at(0);

            cloud->points[i].y = t_lidar_points->at(i).at(1).at(0);

            cloud->points[i].z = t_lidar_points->at(i).at(2).at(0);

            cloud->points[i].r = int(distr_x(eng_x));

            cloud->points[i].g = int(distr_y(eng_y));

            cloud->points[i].b = int(distr_z(eng_z));
        }

        pcl::visualization::PCLVisualizer viewer("raw point cloud");

        viewer.addPointCloud(cloud, "cloud");

        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

        viewer.setBackgroundColor(0.0, 0.0, 0.0);

        viewer.spin();

        viewer.close();
    }

    void K_Means::clusteredCloudVisualizationThread(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const size_t &t_cluster_number)
    {
        pcl::visualization::PCLVisualizer viewer("clustered point cloud");

        viewer.addPointCloud(cloud, "cloud");

        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

        viewer.setBackgroundColor(0.0, 0.0, 0.0);

        viewer.spin();

        viewer.close();
    }

    void K_Means::callRawData(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {
        std::thread t1(&K_Means::rawpointCloudVisualizationThread, this, t_lidar_points);

        t1.detach();
    }
}
