#include "navigation/trajectory.hpp"

void Trajectory::loadFromFile(const std::string &filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        // SD Card not in
        return;
        // throw std::runtime_error("Failed to open file: " + filepath);
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