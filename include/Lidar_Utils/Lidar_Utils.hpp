#ifndef _LIDAR_UTILS_H_
#define _LIDAR_UTILS_H_

// C++ System Headers
#include <iostream>
#include <unordered_map>
#include <vector>
#include <array>
#include <thread>
#include <memory>
#include <random>

namespace Lidar_Simulation
{
    class Lidar_Utils

    {
    public:
        Lidar_Utils();

        ~Lidar_Utils();

        virtual void lidarPointsPrinter(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size);
    };

}

#endif //_LIDAR_UTILS_H_
