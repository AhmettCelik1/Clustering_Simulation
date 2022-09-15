#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Tool_Option/Lidar_Tool_Option.hpp"

namespace Lidar_Simulation
{
    Lidar_Tool_Option::Lidar_Tool_Option()
        : m_lidar_points{nullptr}, m_points_x{nullptr}, m_points_y{nullptr}, m_points_z{nullptr},
          m_points_range{{"x_min", 0}, {"x_max", 10}, {"y_min", -3}, {"y_max", 3}, {"z_min", -1}, {"z_max", 3}},
          frame_id{"velodyne"},
          Lidar_Utils(),
          eng_x(rd_x()), eng_y(rd_y()), eng_z(rd_z()),
          distr_x(m_points_range["x_min"], m_points_range["x_max"]),
          distr_y(m_points_range["y_min"], m_points_range["y_max"]),
          distr_z(m_points_range["z_min"], m_points_range["z_max"])
    {

        std::cout << "Lidar_Tool_Option constructor is called" << std::endl;

        m_lidar_points = std::make_shared<std::vector<std::array<double, 3>>>();

        m_points_x = new std::array<double, 1>();

        m_points_y = new std::array<double, 1>();

        m_points_z = new std::array<double, 1>();
    }
    Lidar_Tool_Option::~Lidar_Tool_Option()
    {
        std::cout << "Lidar_Tool_Option destructor is called" << std::endl;

        delete[] m_points_x;

        delete[] m_points_y;

        delete[] m_points_z;
    }

    std::shared_ptr<std::vector<std::array<double, 3>>> Lidar_Tool_Option::generatorLidarPoints(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size)
    {
        for (size_t i{0}; i < t_size; ++i)
        {

            m_points_x->at(0) = distr_x(eng_x);
            m_points_y->at(0) = distr_y(eng_y);
            m_points_z->at(0) = distr_z(eng_z);

            t_lidar_points->push_back({m_points_x->at(0), m_points_y->at(0), m_points_z->at(0)});
        }

        return t_lidar_points;
    }

    std::shared_ptr<std::vector<std::array<double, 3>>> Lidar_Tool_Option::switcherLidarSize(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points)
    {
        std::cout << "Please Enter the Lidar Size: ";
        std::cin >> m_size;

        for (size_t i{0}; i < m_size; ++i)
        {

            std::srand(std::time(nullptr));

            m_points_x->at(0) = m_points_range["x_min"] + (m_points_range["x_max"] - m_points_range["x_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_x(eng_x);
            m_points_y->at(0) = m_points_range["y_min"] + (m_points_range["y_max"] - m_points_range["y_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_y(eng_y);
            m_points_z->at(0) = m_points_range["z_min"] + (m_points_range["z_max"] - m_points_range["z_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_z(eng_z);

            if (m_points_x->at(0) > m_points_range["x_max"] || m_points_x->at(0) < m_points_range["x_min"] || m_points_y->at(0) > m_points_range["y_max"] || m_points_y->at(0) < m_points_range["y_min"] || m_points_z->at(0) > m_points_range["z_max"] || m_points_z->at(0) < m_points_range["z_min"])
            {

                m_points_x->at(0) = +std::log(m_points_range["x_min"] + (m_points_range["x_max"] - m_points_range["x_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_x(eng_x));
                m_points_y->at(0) = std::log(m_points_range["y_min"] + (m_points_range["y_max"] - m_points_range["y_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_y(eng_y));
                m_points_z->at(0) = std::log(m_points_range["z_min"] + (m_points_range["z_max"] - m_points_range["z_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_z(eng_z));

                // std::cout << "IN LOG CONTROL" << std::endl;

                t_lidar_points->push_back({m_points_x->at(0), m_points_y->at(0), m_points_z->at(0)});
            }
            else

            {   
                m_points_x->at(0) = m_points_range["x_min"] + (m_points_range["x_max"] - m_points_range["x_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_x(eng_x);
                m_points_y->at(0) = m_points_range["y_min"] + (m_points_range["y_max"] - m_points_range["y_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_y(eng_y);
                m_points_z->at(0) = m_points_range["z_min"] + (m_points_range["z_max"] - m_points_range["z_min"]) * (std::rand() / (RAND_MAX + 1.0)) + distr_z(eng_z);

                // std::cout << "NON LOG CONTROL" << std::endl;
                t_lidar_points->push_back({m_points_x->at(0), m_points_y->at(0), m_points_z->at(0)});
            }
        }

        return t_lidar_points;
    }
    void Lidar_Tool_Option::lidarPointsPrinter(const std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points, const size_t &t_size)
    {
        for (size_t i{0}; i < t_size; ++i)
        {
            std::cout << "Point: " << i + 1 << " x: " << t_lidar_points->at(i)[0] << " y: " << t_lidar_points->at(i)[1] << " z: " << t_lidar_points->at(i)[2] << std::endl;
        }
        std::cout << "IN Lidar_Tool_Option" << std::endl;
    }

}