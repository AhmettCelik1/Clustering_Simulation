#include "../../include/Lidar_Tool_Option/Lidar_Tool_Option.hpp"

namespace Lidar_Simulation
{
    Lidar_Tool_Option::Lidar_Tool_Option() : m_points_range{{"x_min", -10}, {"x_max", 45}, {"y_min", -20}, {"y_max", 20}, {"z_min", -5}, {"z_max", 10}}
    {
    }
    Lidar_Tool_Option::~Lidar_Tool_Option()
    {
        std::cout << "Lidar_Tool_Option destructor is called" << std::endl;
    }

    std::shared_ptr<std::vector<std::vector<std::vector<double>>>> Lidar_Tool_Option::generatorLidarPoints(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points, const size_t &t_size)
    {

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;

        std::uniform_real_distribution<double> distr_x;

        std::uniform_real_distribution<double> distr_y;

        std::uniform_real_distribution<double> distr_z;

        std::random_device rd_x;

        std::random_device rd_y;

        std::random_device rd_z;

        std::mt19937 eng_x(rd_x());

        std::mt19937 eng_y(rd_y());

        std::mt19937 eng_z(rd_z());

        distr_x = std::uniform_real_distribution<double>(m_points_range["x_min"], m_points_range["x_max"]);

        distr_y = std::uniform_real_distribution<double>(m_points_range["y_min"], m_points_range["y_max"]);

        distr_z = std::uniform_real_distribution<double>(m_points_range["z_min"], m_points_range["z_max"]);

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