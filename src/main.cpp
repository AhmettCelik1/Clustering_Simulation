#include "../include/Clustering_Activation/Clustering_Activation.hpp"
#include "../include/Lidar_Utils/Lidar_Utils.hpp"

int main()
{
    size_t t_size{};

    std::shared_ptr<Clustering_Simulation::Clustering_Activation> lidar_activation{nullptr};

    lidar_activation = std::make_shared<Clustering_Simulation::Clustering_Activation>(t_size);

    lidar_activation;
    return 0;
}