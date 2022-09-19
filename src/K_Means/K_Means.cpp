#include "../../include/K_Means/K-Means.hpp"

namespace Lidar_Simulation
{
    K_Means::K_Means() : m_points_color_range{{"r_min", 0},
                                              {"r_max", 255},
                                              {"g_min", 0},
                                              {"g_max", 255},
                                              {"b_min", 0},
                                              {"b_max", 255}},
                         m_flag{false},
                         eng_r(rd_r()), eng_g(rd_g()), eng_b(rd_b())
    {

        distr_r = std::uniform_real_distribution<double>(m_points_color_range["r_min"], m_points_color_range["r_max"]);

        distr_g = std::uniform_real_distribution<double>(m_points_color_range["g_min"], m_points_color_range["g_max"]);

        distr_b = std::uniform_real_distribution<double>(m_points_color_range["b_min"], m_points_color_range["b_max"]);
    }

    K_Means::~K_Means()
    {
        std::cout << "K_Means destructor is called" << std::endl;
    }

    void K_Means::pclPointCloudConverter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud)
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

    void K_Means::kMeansClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_cluster_number)
    {

        m_flag = true;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud->width = t_lidar_points->size();

        cloud->height = 1;

        cloud->points.resize(cloud->width * cloud->height);

        pclPointCloudConverter(t_lidar_points, cloud);

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

        pcl::Kmeans::Centroids centroids = real.get_centroids();

        for (size_t i{0}; i < centroids.size(); i++)
        {

            printf("Centroid x: %f, Centroid y: %f, Centroid z: %f \n", centroids[i][0], centroids[i][1], centroids[i][2]);
        }

        printf("points in total cloud: %d \t", static_cast<int>(cloud->size()));
        printf("cluster number: %d \t", static_cast<int>(t_cluster_number));
        printf("centroids size: %d \t", static_cast<int>(centroids.size()));

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        centroid_cloud->width = centroids.size();

        centroid_cloud->height = 1;

        centroid_cloud->points.resize(centroid_cloud->width * centroid_cloud->height);

        for (size_t i = 0; i < centroid_cloud->points.size(); ++i)
        {

            centroid_cloud->points[i].x = centroids[i][0];

            centroid_cloud->points[i].y = centroids[i][1];

            centroid_cloud->points[i].z = centroids[i][2];

            centroid_cloud->points[i].r = int(distr_r(eng_r));

            centroid_cloud->points[i].g = int(distr_g(eng_g));

            centroid_cloud->points[i].b = int(distr_b(eng_b));
        }

        m_cloud_cluster = centroid_cloud;
        m_lidar_points = t_lidar_points;
        m_cluster_number = t_cluster_number;

        // m_cluster_thread = std::thread(&K_Means::clusteredCloudVisualizationThread, this);

        // m_cluster_thread.detach();

        clusteredCloudVisualizationThread();
    }

    void K_Means::clusteredCloudVisualizationThread()
    {
        pcl::visualization::PCLVisualizer viewer("clustered point cloud");

        viewer.addPointCloud(m_cloud_cluster, "cloud");

        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

        viewer.setBackgroundColor(0.0, 0.0, 0.0);

        viewer.addCoordinateSystem(1, 0, 0, 0);
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << " Inorder to go back menu click on the viewer window and press '❌' " << std::endl;

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        m_flag = false;

        viewer.close();

        //-----------------------------------------------------------------------

        // pcl::visualization::CloudViewer cloud_viewer("clustered point cloud");

        // cloud_viewer.showCloud(m_cloud_cluster);
    }

    void K_Means::callRawData(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pclPointCloudConverter(t_lidar_points, cloud);

        m_cloud = cloud;

        // m_raw_thread = std::thread(&K_Means::rawpointCloudVisualizationThread, this);

        // m_raw_thread.detach();

        rawpointCloudVisualizationThread();
    }

    void K_Means::rawpointCloudVisualizationThread()
    {

        pcl::visualization::PCLVisualizer viewer("raw point cloud");

        viewer.addPointCloud(m_cloud, "cloud");

        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

        viewer.setBackgroundColor(0.0, 0.0, 0.0);

        viewer.addCoordinateSystem(1, 0, 0, 0);

        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << " Inorder to go back menu click on the viewer window and press '❌' " << std::endl;

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        m_flag = false;

        viewer.close();

        //-----------------------------------------------------------------------

        // pcl::visualization::CloudViewer cloud_viewer("raw point cloud");

        // cloud_viewer.showCloud(m_cloud);
    }

}
