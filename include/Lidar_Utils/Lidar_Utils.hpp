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

#include "../../include/Lidar_Tool_Option/Lidar_Tool_Option.hpp"

namespace Lidar_Simulation
{
    class Lidar_Utils : protected Lidar_Tool_Option

    {
    public:
        Lidar_Utils();

        virtual ~Lidar_Utils();

        std::unordered_map<std::string, double> m_points_range;

        std::uniform_real_distribution<double> distr_x{};

        std::uniform_real_distribution<double> distr_y{};

        std::uniform_real_distribution<double> distr_z{};

        std::random_device rd_x{};

        std::random_device rd_y{};

        std::random_device rd_z{};

        std::default_random_engine eng_x{};

        std::default_random_engine eng_y{};

        std::default_random_engine eng_z{};

        Lidar_Tool_Option m_lidar_tool_option;

        size_t m_size;

        double m_percantage;

        virtual void lidarPointsPrinter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points) override;

        virtual std::shared_ptr<std::vector<std::vector<std::vector<double>>>> sizeIncreaser(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points) final;

        virtual std::shared_ptr<std::vector<std::vector<std::vector<double>>>> sizeDecreaser(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points) final;
    };

}

#endif //_LIDAR_UTILS_H_
