#include "../../include/Euclidean_Clustering/Euclidean_Clustering.hpp"

namespace Clustering_Simulation
{
    Euclidean_Clustering::Euclidean_Clustering() : Clustering_Visualization(),
                                                   m_cloud_name{"Euclidean_Clustering Cloud"}
    {
    }

    Euclidean_Clustering::~Euclidean_Clustering()
    {
        printf("Euclidean Clustering Destructor Called \n");
        std::cout << std::endl;
    }

    void Euclidean_Clustering::executeEcludianClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const double t_tolarance)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        this->vectorToPclPointCloudConverter(t_lidar_points, cloud);

        for (auto point : *cloud)
        {
            std::cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << " r: " << point.r << " g: " << point.g << " b: " << point.b << std::endl;
        }

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
            this->clusteringVisualization(cloud, m_cloud_name);
        }
        else
        {
            printf("Number of clusters is equal to %d \n", cluster_indices.size());
            printf("Tolarance is too high \n");
        }
    }

}
