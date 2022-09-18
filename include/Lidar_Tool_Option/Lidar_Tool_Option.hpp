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

namespace Lidar_Simulation
{
  class Lidar_Tool_Option

  {
  public:
    Lidar_Tool_Option();

    virtual ~Lidar_Tool_Option();

    size_t m_size;

    std::unordered_map<std::string, double> m_points_range;

    virtual std::shared_ptr<std::vector<std::vector<std::vector<double>>>> generatorLidarPoints(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_size) final; // prevent a method to be overriden

    virtual void lidarPointsPrinter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points);
  };

}

#endif //_LIDAR_TOOL_OPTION_H_
