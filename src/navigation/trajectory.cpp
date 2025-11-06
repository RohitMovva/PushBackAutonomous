#include "navigation/trajectory.hpp"
#include "utilities/routes.hpp"

void Trajectory::loadFromFile(const std::string &filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        // SD Card not in
        return;
    }

    dataset.clear();
    current_index = 0;

    std::string line;
    double past_time = 0.0;
    double past_linear_vel = 0.0;
    double past_angular_vel = 0.0;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        int type;
        iss >> type;

        if (type == 0)
        {
            TrajectoryPoint tp;
            iss >> tp.time >> tp.x >> tp.y >> tp.theta >> tp.linear_vel >> tp.angular_vel;
            
            // Calculate linear and angular acceleration if available
            tp.linear_accel = (tp.linear_vel - past_linear_vel) / (tp.time - past_time);
            tp.angular_accel = (tp.angular_vel - past_angular_vel) / (tp.time - past_time);

            past_time = tp.time;
            past_linear_vel = tp.linear_vel;
            past_angular_vel = tp.angular_vel;

            dataset.emplace_back(tp);
        }
        else if (type == 1)
        {
            ActionPoint ap;
            float action;
            while (iss >> action)
            {
                ap.actions.push_back(action);
            }
            dataset.emplace_back(ap);
        }
    }
}

// void Trajectory::loadFromVector()
// {
//     dataset.clear();
//     current_index = 0;

//     double past_time = 0.0;
//     double past_linear_vel = 0.0;
//     double past_angular_vel = 0.0;

//     // Iterate through all points in the trajectory_points vector from routes.h
//     for (const auto& point : trajectory_points)
//     {
//         // Check the number of elements to determine the type
//         size_t num_elements = point.size();

//         if (points[0] == 0)
//         {
//             // Type 0: Trajectory point with 7 elements
//             // Format: type, time, x, y, theta, linear_vel, angular_vel
//             TrajectoryPoint tp;
//             tp.time = point[1];
//             tp.x = point[2];
//             tp.y = point[3];
//             tp.theta = point[4];
//             tp.linear_vel = point[5];
//             tp.angular_vel = point[6];

//             // Calculate linear and angular acceleration
//             if (past_time > 0.0)
//             {
//                 double dt = tp.time - past_time;
//                 if (dt > 0.0)
//                 {
//                     tp.linear_accel = (tp.linear_vel - past_linear_vel) / dt;
//                     tp.angular_accel = (tp.angular_vel - past_angular_vel) / dt;
//                 }
//                 else
//                 {
//                     tp.linear_accel = 0.0;
//                     tp.angular_accel = 0.0;
//                 }
//             }
//             else
//             {
//                 tp.linear_accel = 0.0;
//                 tp.angular_accel = 0.0;
//             }

//             past_time = tp.time;
//             past_linear_vel = tp.linear_vel;
//             past_angular_vel = tp.angular_vel;

//             dataset.emplace_back(tp);
//         }
//         else if (points[1] == 1)
//         {
//             // Type 1: Action point with 6 elements
//             // Format: type, action1, action2, action3, action4, action5
//             ActionPoint ap;
//             // Skip the first element (type indicator) and add the rest as actions
//             for (size_t i = 1; i < num_elements; ++i)
//             {
//                 ap.actions.push_back(static_cast<float>(point[i]));
//             }
//             dataset.emplace_back(ap);
//         }
//     }
// }

const DataPoint *Trajectory::getNext()
{
    if (current_index >= dataset.size())
    {
        return nullptr;
    }
    return &dataset[current_index++];
}

const DataPoint &Trajectory::getByIndex(size_t index) const
{
    if (index >= dataset.size())
    {
        throw std::out_of_range("Index out of range");
    }
    return dataset[index];
}