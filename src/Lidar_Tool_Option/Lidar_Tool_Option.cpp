#include "../../include/Lidar_Tool_Option/Lidar_Tool_Option.hpp"

namespace Clustering_Simulation
{
    Lidar_Tool_Option::Lidar_Tool_Option() : m_points_range{{"x_min", -40},
                                                            {"x_max", 40},
                                                            {"y_min", -40},
                                                            {"y_max", 40},
                                                            {"z_min", -40},
                                                            {"z_max", 40}},
                                             eng_x(rd_x()), eng_y(rd_y()), eng_z(rd_z()),
                                             distr_x(m_points_range["x_min"], m_points_range["x_max"]),
                                             distr_y(m_points_range["y_min"], m_points_range["y_max"]),
                                             distr_z(m_points_range["z_min"], m_points_range["z_max"])
    {
    }
    Lidar_Tool_Option::~Lidar_Tool_Option()
    {
        std::cout << "Lidar_Tool_Option destructor is called" << std::endl;
        std::cout << std::endl;
    }

    std::shared_ptr<std::vector<std::vector<std::vector<double>>>> Lidar_Tool_Option::generatorLidarPoints(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_size)
    {

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;

        m_size = t_size;

        t_lidar_points->resize(m_size);

        for (size_t i = 0; i < m_size; i++)
        {
            t_lidar_points->at(i).resize(3);

            for (size_t j = 0; j < 3; j++)
            {
                t_lidar_points->at(i).at(j).resize(1);
            }
        }

        for (size_t i = 0; i < m_size; i++)
        {

            t_lidar_points->at(i).at(0).at(0) = distr_x(eng_x);

            t_lidar_points->at(i).at(1).at(0) = distr_y(eng_y);

            t_lidar_points->at(i).at(2).at(0) = distr_z(eng_z);
        }

        return t_lidar_points;

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;
    }

    void Lidar_Tool_Option::lidarPointsPrinter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;

        for (size_t i = 0; i < m_size; i++)
        {
            std::cout << "x: " << t_lidar_points->at(i).at(0).at(0) << " y: " << t_lidar_points->at(i).at(1).at(0) << " z: " << t_lidar_points->at(i).at(2).at(0) << std::endl;
        }

        std::cout << "Size: " << t_lidar_points->size() << std::endl;

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;
    }

}