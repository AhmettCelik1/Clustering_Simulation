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
  class Lidar_Tool_Option : protected Lidar_Utils

  {
  public:
    Lidar_Tool_Option();

    virtual ~Lidar_Tool_Option();

    std::shared_ptr<std::vector<std::array<double, 3>>> m_lidar_points;

    std::array<double, 1> *m_points_x;

    std::array<double, 1> *m_points_y;

    std::array<double, 1> *m_points_z;

    size_t m_size;

    std::unordered_map<std::string, double> m_points_range;

    const std::string frame_id;

    std::uniform_real_distribution<double> distr_x{};

    std::uniform_real_distribution<double> distr_y{};

    std::uniform_real_distribution<double> distr_z{};

    std::random_device rd_x{};

    std::random_device rd_y{};

    std::random_device rd_z{};

    std::default_random_engine eng_x{};

    std::default_random_engine eng_y{};

    std::default_random_engine eng_z{};

    virtual std::shared_ptr<std::vector<std::array<double, 3>>> generatorLidarPoints(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size) final;

    virtual std::shared_ptr<std::vector<std::array<double, 3>>> switcherLidarSize(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points) final; // prevent a method to be overriden

    virtual void lidarPointsPrinter(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size) override;
  };

}

#endif //_LIDAR_TOOL_OPTION_H_
