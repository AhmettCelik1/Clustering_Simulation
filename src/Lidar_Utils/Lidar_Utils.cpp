#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Utils/Lidar_Utils.hpp"

namespace Lidar_Simulation
{
    Lidar_Utils::Lidar_Utils()
    {

    }

    Lidar_Utils::~Lidar_Utils()
    {

    }

    void Lidar_Utils::lidarPointsPrinter(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size)
    {

        for (size_t i{0}; i < t_size; ++i)
        {
            std::cout << "Point: " << i + 1 << " x: " << t_lidar_points->at(i)[0] << " y: " << t_lidar_points->at(i)[1] << " z: " << t_lidar_points->at(i)[2] << std::endl;
        }
    }
}