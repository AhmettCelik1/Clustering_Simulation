#include "../../include/DBscan_Clustering/DBscan_Clustering.hpp"

namespace Clustering_Simulation
{
    DBscan_Clustering::DBscan_Clustering() : K_Means(),
                                             Clustering_Visualization(),
                                             m_cloud_name{"DBscan_Clustering Cloud"}
    {
        m_k_means = std::make_shared<K_Means>();

        m_clustering_visualization = std::make_shared<Clustering_Visualization>();
    }

    DBscan_Clustering::~DBscan_Clustering()
    {
        printf("Destructor DBscan Clustering is called.\n");

        std::cout << std::endl;
    }

    void DBscan_Clustering::executeDBscanClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const double &t_min_points, const double &t_eps)
    {
        // fill in pcl cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = t_lidar_points->size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i{0}; i < cloud->points.size(); ++i)
        {
            cloud->points[i].x = t_lidar_points->at(i).at(0).at(0);
            cloud->points[i].y = t_lidar_points->at(i).at(1).at(0);
            cloud->points[i].z = t_lidar_points->at(i).at(2).at(0);
        }

        // create kd tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // create vector of point indices
        std::vector<pcl::PointIndices> cluster_indices;

        // create euclidean cluster extraction object
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(t_eps); // 2cm
        ec.setMinClusterSize(t_min_points);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // clusted pcl cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>);

        int j{0};
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {
                pcl::PointXYZRGB point;
                point.x = cloud->points[*pit].x;
                point.y = cloud->points[*pit].y;
                point.z = cloud->points[*pit].z;
                point.r = int(m_k_means->distr_r(m_k_means->eng_r));
                point.g = int(m_k_means->distr_g(m_k_means->eng_g));
                point.b = int(m_k_means->distr_b(m_k_means->eng_b));
                cloud_cluster->points.push_back(point);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            *cloud_clustered += *cloud_cluster;
            ++j;
        }

        if (cloud_clustered->points.size() > 0)
        {
            printf("Number of clusters is equal to %d", j);
            std::cout << std::endl;
        }
        else
        {
            printf("No clusters found in the point cloud data");
            std::cout << std::endl;
            printf("Number of clusters is equal to %d", j);
            std::cout << std::endl;
            printf("Change the parameters of DBscan clustering");
            std::cout << std::endl;
        }

        // Print Clustered Groups Coordinates
        for (size_t i{0}; i < cloud_clustered->points.size(); ++i)
        {
            std::cout << "x: " << cloud_clustered->points[i].x << " y: " << cloud_clustered->points[i].y << " z: " << cloud_clustered->points[i].z << std::endl;
        }
        std::cout << std::endl;

        std::cout << "Number of points in clustered cloud: " << cloud_clustered->points.size() << std::endl;

        m_clustering_visualization->clusteringVisualization(cloud_clustered, m_cloud_name);
    }

} // namespace Lidar_Simulation