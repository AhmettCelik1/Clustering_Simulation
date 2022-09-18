#include "../include/Lidar_Activation/Lidar_Activation.hpp"
#include "../include/Lidar_Utils/Lidar_Utils.hpp"

int main()
{
    size_t t_size{};

    std::shared_ptr<Lidar_Simulation::Lidar_Activation> lidar_activation{nullptr};

    lidar_activation = std::make_shared<Lidar_Simulation::Lidar_Activation>(t_size);

    // size_t size{5};

    // std::shared_ptr<std::vector<std::array<double, 3>>> m_lidar_points{nullptr};

    // m_lidar_points = std::make_shared<std::vector<std::array<double, 3>>>();

    // std::unordered_map<std::string, int> m_x_points_range{{"x_min", 0}, {"x_max", 10}};

    // std::unordered_map<std::string, int> m_y_points_range{{"y_min", -5}, {"y_max", 5}};

    // std::unordered_map<std::string, int> m_z_points_range{{"z_min", -5}, {"z_max", 5}};

    // for (size_t i{0}; i < size; ++i)
    // {

    //     std::random_device rd_x;
    //     std::default_random_engine eng_x(rd_x());

    //     std::random_device rd_y;
    //     std::default_random_engine eng_y(rd_y());

    //     std::random_device rd_z;
    //     std::default_random_engine eng_z(rd_z());

    //     std::uniform_real_distribution<double> distr_x = std::uniform_real_distribution<double>(m_x_points_range["x_min"], m_x_points_range["x_max"]);

    //     std::uniform_real_distribution<double> distr_y = std::uniform_real_distribution<double>(m_y_points_range["y_min"], m_y_points_range["y_max"]);

    //     std::uniform_real_distribution<double> distr_z = std::uniform_real_distribution<double>(m_z_points_range["z_min"], m_z_points_range["z_max"]);

    //     m_lidar_points->push_back({distr_x(eng_x), distr_y(eng_y), distr_z(eng_z)});
    // }

    // Lidar_Simulation::Lidar_Utils *lidar_utils{nullptr};

    // lidar_utils = new Lidar_Simulation::Lidar_Tool_Option();

    // lidar_utils->lidarPointsPrinter(m_lidar_points, size);

    // delete lidar_utils;

    return 0;
}