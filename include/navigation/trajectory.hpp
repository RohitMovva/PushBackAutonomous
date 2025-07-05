#pragma once
#include <variant>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>

/**
 * @brief Represents a trajectory point with pose and velocity information
 */
struct TrajectoryPoint
{
    double time;          ///< Timestamp
    double x;             ///< X position
    double y;             ///< Y position
    double theta;         ///< Orientation angle
    double linear_vel;    ///< Linear velocity
    double angular_vel;   ///< Angular velocity
    double linear_accel;  ///< Linear acceleration (optional, default 0)
    double angular_accel; ///< Angular acceleration (optional, default
};

/**
 * @brief Represents an action point containing multiple action values
 */
struct ActionPoint
{
    std::vector<float> actions; ///< Vector of action values
};

/**
 * @brief Variant type that can hold either a TrajectoryPoint or ActionPoint
 */
using DataPoint = std::variant<TrajectoryPoint, ActionPoint>;

/**
 * @brief Class for loading and managing robot trajectory and action data
 *
 * This class provides functionality to load robot data from a text file and
 * access it either sequentially or by index. The data format supports two
 * types of lines: trajectory points (type 0) and action points (type 1).
 */
class Trajectory
{
private:
    std::vector<DataPoint> dataset; ///< Container for all data points
    size_t current_index = 0;       ///< Current position for sequential access

public:
    /**
     * @brief Load data from a text file
     *
     * Parses a text file where each line starts with 0 (trajectory) or 1 (action).
     * Trajectory lines: time, x, y, theta, linear_vel, angular_vel
     * Action lines: variable number of float values
     *
     * @param filepath Path to the data file
     * @throws std::runtime_error if file cannot be opened
     */
    void loadFromFile(const std::string &filepath);

    /**
     * @brief Get the next data point in sequence
     *
     * Returns a pointer to the next data point and advances the internal index.
     * Used for sequential processing of the dataset.
     *
     * @return Pointer to the next DataPoint, or nullptr if at end
     */
    const DataPoint *getNext();

    /**
     * @brief Get a data point by its index
     *
     * Provides random access to any data point in the dataset.
     * This method is intended for rare use cases.
     *
     * @param index Index of the desired data point
     * @return Reference to the DataPoint at the specified index
     * @throws std::out_of_range if index is invalid
     */
    const DataPoint &getByIndex(size_t index) const;

    /**
     * @brief Get the total number of data points
     * @return Number of data points in the dataset
     */
    size_t size() const { return dataset.size(); }

    /**
     * @brief Check if there are more data points to process
     * @return true if there are more data points, false otherwise
     */
    bool hasNext() const { return current_index < dataset.size(); }

    /**
     * @brief Reset the sequential access index to the beginning
     */
    void reset() { current_index = 0; }

    /**
     * @brief Check if the dataset is empty
     * @return true if no data points are loaded, false otherwise
     */
    bool empty() const { return dataset.empty(); }
};