#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Activation/Lidar_Activation.hpp"

namespace Lidar_Simulation
{

    // Static variable initialization
    size_t Lidar_Activation::m_number_objects{0};

    Lidar_Activation::Lidar_Activation(const size_t &t_size)
        : m_size{t_size}, Lidar_Utils()
    {
        std::cout << "[" << __APP_NAME__ << "] Constructor is called." << std::endl;

        m_lidar_points = std::make_shared<std::vector<std::array<double, 3>>>();

        m_lidar_points = lidarPointsCreater(m_lidar_points);

        m_lidar_utils.lidarPointsPrinter(m_lidar_points, m_size);

        ++m_number_objects;
        displayActiveObjects();
    }

    Lidar_Activation::~Lidar_Activation()
    {
        --m_number_objects;

        std::cout << "[" << __APP_NAME__ << "] Destructor is called." << std::endl;
    }
    inline size_t Lidar_Activation::objectCounter()
    {
        return m_number_objects;
    }

    inline void Lidar_Activation::displayActiveObjects() const
    {

        printf("[%s] There are %zu active objects.\n", __APP_NAME__, m_number_objects);
    }

    inline std::shared_ptr<std::vector<std::array<double, 3>>> Lidar_Activation::lidarPointsCreater(std::shared_ptr<std::vector<std::array<double, 3>>> &t_lidar_points)
    {

        std::random_device rd_x;
        std::default_random_engine eng_x(rd_x());
        std::uniform_real_distribution<float> distr_x(0, 10);

        std::random_device rd_y;
        std::default_random_engine eng_y(rd_y());
        std::uniform_real_distribution<float> distr_y(-5, 5);

        std::random_device rd_z;
        std::default_random_engine eng_z(rd_z());
        std::uniform_real_distribution<float> distr_z(-5, 5);

        for (size_t i{0}; i < m_size; ++i)
        {
            std::array<double, 3> point;

            point[0] = distr_x(eng_x);
            point[1] = distr_y(eng_y);
            point[2] = distr_z(eng_z);

            t_lidar_points->push_back(point);
        }
        return t_lidar_points;
    }

}