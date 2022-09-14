#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Activation/Lidar_Activation.hpp"

namespace Lidar_Simulation
{

    // Static variable initialization
    size_t Lidar_Activation::m_number_objects{0};

    Lidar_Activation::Lidar_Activation(const size_t &t_size)
        : m_size{t_size},
          Lidar_Tool_Option(),
          m_flag{false}

    {

        std::cout << "[" << __APP_NAME__ << "] Constructor is called." << std::endl;

        std::cout << "Enter the size of the lidar points: ";
        std::cin >> m_size;

        m_lidar_points = std::make_shared<std::vector<std::array<double, 3>>>();

        m_lidar_tool_option.generatorLidarPoints(m_lidar_points, m_size);

        m_lidar_utils.lidarPointsPrinter(m_lidar_points, m_size);

        // new std::thread(&Lidar_Activation::threadLidar, this);

        do
        {

            std::cout << "[" << __APP_NAME__ << "] Please select an option from the menu below:" << std::endl;

            std::cout << "[" << __APP_NAME__ << "] 1. Change size of the lidar" << std::endl;

            std::cout << "[ " << __APP_NAME__ << "] 2. Exit " << std::endl;

            std::cout << "-----------Option----------->";

            std::cin >> m_options;
            system("cls");
            system("clear");

            if (m_options == 1)
            {
                m_lidar_points == m_lidar_tool_option.switcherLidarSize(m_lidar_points);
                m_lidar_utils.lidarPointsPrinter(m_lidar_points, m_lidar_tool_option.m_size);
                std::cout << std::endl;
            }
            else if (m_options == 2)
            {
                std::cout << "-----------------Exited------------" << std::endl;
                std::cout << std::endl;
            }
        } while (m_options != 2);

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

}
