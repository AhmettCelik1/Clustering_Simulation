#ifndef _LIDAR_TOOL_OPTION_H_
#define _LIDAR_TOOL_OPTION_H_

// C++ System Headers
#include <iostream>
#include <unordered_map>
#include <vector>
#include <array>
#include <thread>
#include <memory>
#include <random>

#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Utils/Lidar_Utils.hpp"

namespace Lidar_Simulation
{
    class Lidar_Tool_Option : public Lidar_Utils

    {
    public:
        Lidar_Tool_Option();

        ~Lidar_Tool_Option();

        std::shared_ptr<std::vector<std::array<double, 3>>> m_lidar_points;

        std::array<double, 1> *m_points_x;

        std::array<double, 1> *m_points_y;

        std::array<double, 1> *m_points_z;

        size_t m_size;

        std::unordered_map<std::string, int> m_x_points_range;

        std::unordered_map<std::string, int> m_y_points_range;

        std::unordered_map<std::string, int> m_z_points_range;

        const std::string frame_id;

        std::shared_ptr<std::vector<std::array<double, 3>>> generatorLidarPoints(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size);

        std::shared_ptr<std::vector<std::array<double, 3>>> switcherLidarSize(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points);

        virtual void lidarPointsPrinter(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size) override;
    };

}

#endif //_LIDAR_TOOL_OPTION_H_
