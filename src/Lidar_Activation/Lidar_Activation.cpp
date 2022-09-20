#include "../../include/Lidar_Activation/Lidar_Activation.hpp"

namespace Lidar_Simulation
{

    // Static variable initialization
    size_t Lidar_Activation::m_number_objects{0};

    Lidar_Activation::Lidar_Activation(const size_t &t_size)
        : m_size{t_size},
          Lidar_Utils(),
          K_Means(),
          m_lidar_points_3d{nullptr},
          m_flag{false}

    {
        std::cout << "[" << __APP_NAME__ << "] Constructor Lidar Activation  is called." << std::endl;

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

            std::cout << " [" << __APP_NAME__ << "] 3. Execute K Means Clustering " << std::endl;

            std::cout << "[" << __APP_NAME__ << "] 4. Execute Ecludian Clustering " << std::endl;

            std::cout << "[" << __APP_NAME__ << "] 5. Execute DBscan Clustering " << std::endl;

            std::cout << "[" << __APP_NAME__ << "] 6. Print Lidar Points " << std::endl;

            std::cout << "[ " << __APP_NAME__ << "] 7. Exit " << std::endl;

            std::cout << "-----------Option----------->";

            std::cin >> m_options;

            system("clear");

            if (m_options == 1)
            {

                m_lidar_points_3d = m_lidar_utils.sizeIncreaser(m_lidar_points_3d);
                std::cout << "THE SIZE OF THE LIDAR----->" << m_lidar_points_3d->size() << std::endl;
                m_lidar_utils.lidarPointsPrinter(m_lidar_points_3d);
                std::cout << std::endl;
            }

            else if (m_options == 2)
            {
                m_lidar_points_3d = m_lidar_utils.sizeDecreaser(m_lidar_points_3d);
                std::cout << "THE SIZE OF THE LIDAR----->" << m_lidar_points_3d->size() << std::endl;
                m_lidar_utils.lidarPointsPrinter(m_lidar_points_3d);
                std::cout << std::endl;
            }

            else if (m_options == 3)
            {

                std::cout << "[" << __APP_NAME__ << "] 1. Visulize Raw Data" << std::endl;

                std::cout << "[" << __APP_NAME__ << "] 2. Execute K-Means Clustering" << std::endl;

                std::cout << "-----------Option----------->";

                std::cin >> m_options;

                if (m_options == 1)
                {
                    m_k_means.callRawData(m_lidar_points_3d);
                }
                else if (m_options == 2)
                {
                    std::cout << "Enter the number of clusters: ";
                    std::cin >> m_cluster_number;
                    m_k_means.kMeansClustering(m_lidar_points_3d, m_cluster_number);
                }
                std::cout << std::endl;
            }

            else if (m_options == 4)
            {
                std::cout << "enter a tolerance value: ";
                std::cin >> m_tolerance;

                m_euclidean_clustering.executeEcludianClustering(m_lidar_points_3d, m_tolerance);
            }

            else if (m_options == 5)
            {
                
                std::cout << "Enter the minimum number of points in a cluster: ";
                std::cin >> m_min_points;
                std::cout << "Enter the maximum distance between two points: ";
                std::cin >> m_eps;

                m_dbscan_clustering.executeDBscanClustering(m_lidar_points_3d, m_min_points, m_eps);
            }

            else if (m_options == 6)
            {
                m_lidar_utils.lidarPointsPrinter(m_lidar_points_3d);
                std::cout << "THE SIZE OF THE LIDAR----->" << m_lidar_points_3d->size() << std::endl;
                std::cout << std::endl;
            }

            else if (m_options == 7)
            {
                std::cout << "-----------------Exited------------" << std::endl;

                std::cout << std::endl;
            }

            else
            {
                std::cout << "-----------------Invalid Option------------" << std::endl;

                std::cout << std::endl;
            }
        } while (m_options != 7);
    }

}
