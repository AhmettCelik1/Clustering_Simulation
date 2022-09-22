#include "../../include/Lidar_Utils/Lidar_Utils.hpp"

namespace Clustering_Simulation
{
    Lidar_Utils::Lidar_Utils() : Lidar_Tool_Option()
    {
        m_lidar_tool_option = std::make_shared<Lidar_Tool_Option>();
    }

    Lidar_Utils::~Lidar_Utils()
    {
        std::cout << "Lidar_Utils destructor is called" << std::endl;
        std::cout << std::endl;
    }

    void Lidar_Utils::lidarPointsPrinter(const std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;

        for (size_t i{0}; i < t_lidar_points->size(); ++i)
        {
            std::cout << "x: " << t_lidar_points->at(i).at(0).at(0) << " y: " << t_lidar_points->at(i).at(1).at(0) << " z: " << t_lidar_points->at(i).at(2).at(0) << std::endl;
        }

        std::cout << "Size of the points: " << t_lidar_points->size() << std::endl;

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;
    }

    std::shared_ptr<std::vector<std::vector<std::vector<double>>>> Lidar_Utils::sizeIncreaser(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;

        std::cout << "By what percentage do you want to increase the density?" << std::endl;

        std::cout << "Percantage--->>";

        std::cin >> m_percantage;

        m_size = t_lidar_points->size();

        m_size = m_size + (m_size * m_percantage / 100);

        t_lidar_points->resize(m_size);

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;

        t_lidar_points = m_lidar_tool_option->generatorLidarPoints(t_lidar_points, m_size);

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;
        return t_lidar_points;
    }

    std::shared_ptr<std::vector<std::vector<std::vector<double>>>> Lidar_Utils::sizeDecreaser(std::shared_ptr<std::vector<std::vector<std::vector<double>>>> &t_lidar_points)
    {
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;

        std::cout << "By what percentage do you want to decrease the density?" << std::endl;

        std::cout << "%";

        std::cin >> m_percantage;

        const size_t t_size{t_lidar_points->size()};

        m_size = t_lidar_points->size();

        m_size = m_size - (m_size * m_percantage / 100);

        if (m_size <= 0)
        {
            t_lidar_points->resize(t_size);

            t_lidar_points = m_lidar_tool_option->generatorLidarPoints(t_lidar_points, m_size);

            return t_lidar_points;
        }
        else
        {
            t_lidar_points->resize(m_size);

            t_lidar_points = m_lidar_tool_option->generatorLidarPoints(t_lidar_points, m_size);

            return t_lidar_points;
        }

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << std::endl;
    }

}