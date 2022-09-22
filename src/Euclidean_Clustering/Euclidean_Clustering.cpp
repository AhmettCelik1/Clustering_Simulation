#include "../../include/Euclidean_Clustering/Euclidean_Clustering.hpp"

#include <pcl/segmentation/extract_clusters.h>

namespace Clustering_Simulation
{
    Euclidean_Clustering::Euclidean_Clustering() : K_Means()
    {

        m_k_means = std::make_shared<K_Means>();
    }

    Euclidean_Clustering::~Euclidean_Clustering()
    {
        printf("Euclidean Clustering Destructor Called \n");
        std::cout << std::endl;
    }

    void Euclidean_Clustering::executeEcludianClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const double t_tolarance)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        m_k_means->pclPointCloudConverter(t_lidar_points, cloud);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(t_tolarance); // 2cm
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                cloud_cluster->points.push_back(cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            j++;
        }

        if (cluster_indices.size() > 0)
        {
            printf("Number of clusters is equal to %d \n", cluster_indices.size());
            viewerClusterCloud(cloud);
        }
        else
        {
            printf("Number of clusters is equal to %d \n", cluster_indices.size());
            printf("Tolarance is too high \n");
        }
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

        viewer.close();
    }
}
