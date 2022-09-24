#include "../../include/K_Means/K-Means.hpp"

namespace Clustering_Simulation
{
    K_Means::K_Means() : Clustering_Visualization(),
                         m_kmeans_name("K-Means Clustering Cloud")
    {

        m_clustering_visualization = std::make_shared<Clustering_Visualization>();
    }

    K_Means::~K_Means()
    {
        std::cout << "K_Means destructor is called" << std::endl;
        std::cout << std::endl;
    }

    void K_Means::kMeansClustering(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_cluster_number)
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        m_clustering_visualization->vectorToPclPointCloudConverter(t_lidar_points, cloud);

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

        m_clustering_visualization->clusteringVisualization(centroid_cloud, m_kmeans_name);
    }

}
