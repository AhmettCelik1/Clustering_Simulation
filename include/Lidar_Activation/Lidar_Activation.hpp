#ifndef _LIDAR_ACTIVATION_H_
#define _LIDAR_ACTIVATION_H_

// C++ System Headers
#include <iostream>
#include <unordered_map>
#include <vector>
#include <array>
#include <thread>
#include <memory>
#include <random>
#include <chrono>

#include "../../include/Lidar_Utils/Lidar_Utils.hpp"
#include "../../include/Lidar_Tool_Option/Lidar_Tool_Option.hpp"
#include "../../include/K_Means/K-Means.hpp"
#include "../../include/Euclidean_Clustering/Euclidean_Clustering.hpp"
#include "../../include/DBscan_Clustering/DBscan_Clustering.hpp"


#define __APP_NAME__ "Lidar_Activation"

namespace Lidar_Simulation
{

    class Lidar_Activation final : protected Lidar_Utils, protected K_Means, protected Euclidean_Clustering ,protected  DBscan_Clustering // prevent Lidar_Activation class be derived from
    {
        friend std::ostream &operator<<(std::ostream &os, const Lidar_Activation &lidar_activation);

    public:
        Lidar_Activation(const size_t &t_size);

        virtual ~Lidar_Activation();

    private:
        size_t m_size;

        int m_options;

        //! Get the number of objects.
        static size_t m_number_objects;

        std::shared_ptr<std::vector<std::vector<std::vector<double>>>> m_lidar_points_3d;

        Lidar_Utils m_lidar_utils;

        Lidar_Tool_Option m_lidar_tool_option;

        K_Means m_k_means;

        Euclidean_Clustering m_euclidean_clustering;

        DBscan_Clustering m_dbscan_clustering;

        size_t m_cluster_number;

        double m_min_points;

        double m_eps;

        double m_tolerance;

        bool m_flag{};
        /*!
         * CounterObjects.
         * @return size of how many object is created  if successful.
         */
        static inline size_t objectCounter();

        /*!
         * DisplayActiveObjects.
         * @return  if successful.
         */
        inline void displayActiveObjects() const;

        void threadLidar();
    };
} // Namespace  Lidar_Simulation

#endif // _LIDAR_ACTIVATION_H_
