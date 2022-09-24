#ifndef _CLUSTERING_ACTIVATION_HPP_
#define _CLUSTERING_ACTIVATION_HPP_

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
#include "../../include/Clustering_Visualization/Clustering_Visualization.hpp"

#define __APP_NAME__ "Clustering_Activation"

namespace Clustering_Simulation
{

    class Clustering_Activation final : public Lidar_Utils, // prevent Lidar_Activation class be derived from implemented final keyword
                                        public Lidar_Tool_Option,
                                        public K_Means,
                                        public Euclidean_Clustering,
                                        public DBscan_Clustering,
                                        public Clustering_Visualization
    {
        friend std::ostream &operator<<(std::ostream &os, const Clustering_Activation &lidar_activation);

    public:
        Clustering_Activation(const size_t &t_size);

        virtual ~Clustering_Activation();

    private:
        size_t m_size;

        int m_options;

        //! Get the number of objects.
        static size_t m_number_objects;

        std::shared_ptr<std::vector<std::vector<std::vector<double>>>> m_lidar_points_3d;

        std::shared_ptr<Lidar_Utils> m_lidar_utils;

        std::shared_ptr<Lidar_Tool_Option> m_lidar_tool_option;

        K_Means m_k_means;

        std::shared_ptr<Euclidean_Clustering> m_euclidean_clustering;

        std::shared_ptr<DBscan_Clustering> m_dbscan_clustering;

        std::shared_ptr<Clustering_Visualization> m_clustering_visualization;

        size_t m_cluster_number;

        int m_min_points;

        double m_eps;

        double m_tolerance;

        std::string m_raw_cloud_name;

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

        inline void showMenu();
    };
} // Namespace  Lidar_Simulation

#endif // _CLUSTERING_ACTIVATION_HPP_
