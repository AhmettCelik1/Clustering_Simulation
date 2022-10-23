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


    }

}
