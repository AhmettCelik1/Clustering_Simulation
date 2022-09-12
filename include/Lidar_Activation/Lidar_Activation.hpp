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


#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Utils/Lidar_Utils.hpp"
#include "/home/ahmet/Workspaces/cpp_ws/Lidar_Simulation/include/Lidar_Tool_Option/Lidar_Tool_Option.hpp"

#define __APP_NAME__ "Lidar_Activation"

namespace Lidar_Simulation
{

    class Lidar_Activation : protected Lidar_Utils, protected Lidar_Tool_Option
    {
        friend std::ostream &operator<<(std::ostream &os, const Lidar_Activation &lidar_activation);

    public:
        Lidar_Activation(const size_t &t_size);

        ~Lidar_Activation();

    private:
        size_t m_size;

        int m_options;

        //! Get the number of objects.
        static size_t m_number_objects;

        std::shared_ptr<std::vector<std::array<double, 3>>> m_lidar_points;

        Lidar_Utils m_lidar_utils;

        Lidar_Tool_Option m_lidar_tool_option;

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
