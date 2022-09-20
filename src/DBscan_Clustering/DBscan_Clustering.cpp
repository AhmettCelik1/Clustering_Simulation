#include "../../include/DBscan_Clustering/DBscan_Clustering.hpp"

namespace Lidar_Simulation
{
    DBscan_Clustering::DBscan_Clustering() : K_Means(),
                                             m_points_color_range{{"r_min", 0},
                                                                  {"r_max", 255},
                                                                  {"g_min", 0},
                                                                  {"g_max", 255},
                                                                  {"b_min", 0},
                                                                  {"b_max", 255}},
                                             eng_r(rd_r()), eng_g(rd_g()), eng_b(rd_b())
    {

        distr_r = std::uniform_real_distribution<double>(m_points_color_range["r_min"], m_points_color_range["r_max"]);

        distr_g = std::uniform_real_distribution<double>(m_points_color_range["g_min"], m_points_color_range["g_max"]);

        distr_b = std::uniform_real_distribution<double>(m_points_color_range["b_min"], m_points_color_range["b_max"]);
    }

    DBscan_Clustering::~DBscan_Clustering()
    {
    }

    void DBscan_Clustering::executeDBscanClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const double &t_min_points, const double &t_eps)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        m_k_means.pclPointCloudConverter(t_lidar_points, cloud);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(t_eps);
        ec.setMinClusterSize(t_min_points);
        ec.setMaxClusterSize(t_min_points * 10);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 0;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_main(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(cloud->points[*pit]); //*

            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;

            j++;

            for (size_t i{0}; i < cloud_cluster->points.size(); ++i)
            {
                cloud_cluster->points[i].r = distr_r(eng_r);
                cloud_cluster->points[i].g = distr_g(eng_g);
                cloud_cluster->points[i].b = distr_b(eng_b);
            }

            if (cluster_indices.size() == 1)
            {
                cloud_cluster_main = cloud_cluster;
            }
            else
            {
                *cloud_cluster_main += *cloud_cluster;
            }
        }

        for (size_t i{0}; i < cloud_cluster_main->size(); ++i)
        {
            printf("x = %f, y = %f, z = %f \n", cloud_cluster_main->points[i].x, cloud_cluster_main->points[i].y, cloud_cluster_main->points[i].z);
            printf("size of cloud_cluster_main = %d \n", cloud_cluster_main->size());
        }
        

        clusteredCloudVisualization(cloud_cluster_main);
    }
    void DBscan_Clustering::clusteredCloudVisualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud)
    {

        pcl::visualization::PCLVisualizer viewer("DBscan Clustered Cloud");

        viewer.addPointCloud(t_cloud, "cloud");

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
    }

} // namespace Lidar_Simulation