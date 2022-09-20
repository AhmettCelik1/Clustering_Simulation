#include "../../include/Euclidean_Clustering/Euclidean_Clustering.hpp"

#include <pcl/segmentation/extract_clusters.h>

namespace Lidar_Simulation
{
    Euclidean_Clustering::Euclidean_Clustering() : m_points_color_range{{"r_min", 0},
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

    Euclidean_Clustering::~Euclidean_Clustering()
    {
    }

    void Euclidean_Clustering::executeEcludianClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const double t_tolarance)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        cloud->width = t_lidar_points->size();

        cloud->height = 1;

        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i{0}; i < t_lidar_points->size(); ++i)
        {
            cloud->points[i].x = t_lidar_points->at(i).at(0).at(0);
            cloud->points[i].y = t_lidar_points->at(i).at(1).at(0);
            cloud->points[i].z = t_lidar_points->at(i).at(2).at(0);
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

        ec.setClusterTolerance(t_tolarance);

        ec.setMinClusterSize(10);

        ec.setMaxClusterSize(25000);

        ec.setSearchMethod(tree);

        ec.setInputCloud(cloud);

        ec.extract(cluster_indices);

        int j = 0;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster_main(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                cloud_cluster->points.push_back(cloud->points[*pit]);
            }

            cloud_cluster->width = cloud_cluster->points.size();

            cloud_cluster->height = 1;

            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;

            std::stringstream ss;

            ss << "cloud_cluster_" << j << ".pcd";

            pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster);

            std::cout << "Saved " << cloud_cluster->points.size() << " data points to " << ss.str() << std::endl;

            j++;

            for (size_t i{0}; i < cloud_cluster->points.size(); ++i)
            {
                pcl::PointXYZRGB point;

                point.x = cloud_cluster->points[i].x;

                point.y = cloud_cluster->points[i].y;

                point.z = cloud_cluster->points[i].z;

                point.r = distr_r(eng_r);

                point.g = distr_g(eng_g);

                point.b = distr_b(eng_b);

                cloud_cluster_main->points.push_back(point);
            }

            if (cluster_indices.size() == 1)
            {
                cloud_cluster_main->width = cloud_cluster_main->points.size();
            }
            else
            {
                cloud_cluster_main->width = cloud_cluster_main->points.size() / cluster_indices.size();
            }
        }
        viewerClusterCloud(cloud_cluster_main);
    }

    void Euclidean_Clustering::viewerClusterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &t_cloud)
    {

        pcl::visualization::PCLVisualizer viewer("Euclidean_Clustering Point Cloud");

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
}
