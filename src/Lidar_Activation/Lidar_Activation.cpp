#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Activation/Lidar_Activation.hpp"

namespace Lidar_Simulation
{

    // Static variable initialization
    size_t Lidar_Activation::m_number_objects{0};

    Lidar_Activation::Lidar_Activation(const size_t &t_size)
        : m_size{t_size},
          Lidar_Utils(),
          m_flag{false}

    {
        std::cout << "[" << __APP_NAME__ << "] Constructor Lidar Activation  is called." << std::endl;

        m_lidar_points = std::make_shared<std::vector<std::array<double, 3>>>();

        m_lidar_points_3d = std::make_shared<std::vector<std::vector<std::vector<double>>>>();

        std::cout << "Enter the size of the lidar points: ";
        std::cin >> m_size;

        m_lidar_tool_option.generatorLidarPoints(m_lidar_points_3d, m_size);

        m_lidar_tool_option.lidarPointsPrinter(m_lidar_points_3d);

        ++m_number_objects;

        displayActiveObjects();

        std::thread thread_1(&Lidar_Activation::threadLidar, this);
        thread_1.join();
    }

    Lidar_Activation::~Lidar_Activation()
    {

        --m_number_objects;

        std::cout << "[" << __APP_NAME__ << "] Destructor Lidar Activation is called  ." << std::endl;
    }
    inline size_t Lidar_Activation::objectCounter()
    {
        return m_number_objects;
    }

    inline void Lidar_Activation::displayActiveObjects() const
    {

        printf("[%s] There are %zu active objects.\n", __APP_NAME__, m_number_objects);
    }

    void Lidar_Activation::threadLidar()
    {

        do
        {

            std::cout << "[" << __APP_NAME__ << "] Please select an option from the menu below:" << std::endl;

            std::cout << "[" << __APP_NAME__ << "] 1. Increse Density of Lidar Points" << std::endl;

            std::cout << "[" << __APP_NAME__ << "] 2. Decrese Density of Lidar Points" << std::endl;

            std::cout << "[" << __APP_NAME__ << "] 3. Print Lidar Points " << std::endl;

            std::cout << "[ " << __APP_NAME__ << "] 4. Exit " << std::endl;

            std::cout << "-----------Option----------->";

            std::cin >> m_options;

            // system("clear");

            if (m_options == 1)
            {

                m_lidar_points_3d = m_lidar_utils.sizeIncreaser(m_lidar_points_3d);
                std::cout << "THE SIZE OF THE LIDAR----->" << m_lidar_points_3d->size() << std::endl;
                std::cout << std::endl;
            }

            else if (m_options == 2)
            {
                m_lidar_points_3d = m_lidar_utils.sizeDecreaser(m_lidar_points_3d);
                std::cout << "THE SIZE OF THE LIDAR----->" << m_lidar_points_3d->size() << std::endl;
                std::cout << std::endl;
            }

            else if (m_options == 3)
            {
                m_lidar_utils.lidarPointsPrinter(m_lidar_points_3d);
                std::cout << "THE SIZE OF THE LIDAR----->" << m_lidar_points_3d->size() << std::endl;
                std::cout << std::endl;
            }

            else if (m_options == 4)
            {
                std::cout << "-----------------Exited------------" << std::endl;

                std::cout << std::endl;
            }

            else
            {
                std::cout << "-----------------Invalid Option------------" << std::endl;

                std::cout << std::endl;
            }
        } while (m_options != 4);
    }

}
