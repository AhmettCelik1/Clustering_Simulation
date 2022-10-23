#ifndef _LIDAR_TOOL_OPTION_HPP_
#define _LIDAR_TOOL_OPTION_HPP_

// C++ System Headers
#include <iostream>
#include <unordered_map>
#include <vector>
#include <array>
#include <thread>
#include <memory>
#include <random>

namespace Clustering_Simulation
{
  class Lidar_Tool_Option

  {
  public:
    Lidar_Tool_Option();

    virtual ~Lidar_Tool_Option();

    size_t m_size;

    std::unordered_map<std::string, double> m_points_range;

    std::uniform_real_distribution<double> distr_x;

    std::uniform_real_distribution<double> distr_y;

    std::uniform_real_distribution<double> distr_z;

    std::random_device rd_x;

    std::random_device rd_y;

    std::random_device rd_z;

    std::default_random_engine eng_x;

    std::default_random_engine eng_y;

    std::default_random_engine eng_z;

    virtual std::shared_ptr<std::vector<std::vector<std::vector<double>>>> generatorLidarPoints(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_size) final; // prevent a method to be overriden

    virtual void lidarPointsPrinter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points);
  };

}

#endif //_LIDAR_TOOL_OPTION_HPP_
