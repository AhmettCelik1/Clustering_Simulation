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
