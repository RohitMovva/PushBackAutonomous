#ifndef EXPONENTIAL_FILTER_H
#define EXPONENTIAL_FILTER_H

/**
 * @brief Simple exponential filter for smoothing data
 * 
 * Implements a first-order exponential filter (also known as exponential moving average)
 * for smoothing noisy measurements. The smoothing factor alpha determines how much
 * weight is given to new measurements versus previous values.
 */
class ExponentialFilter {
private:
    double alpha;         ///< Smoothing factor (0 < alpha <= 1)
    double prevValue;     ///< Previous filtered value
    bool initialized;     ///< Tracks if filter has received first value

public:
    /**
     * @brief Construct a new Exponential Filter
     * 
     * @param smoothingFactor Alpha value between 0 and 1
     *        Higher values give more weight to new measurements (less smoothing)
     *        Lower values give more weight to previous values (more smoothing)
     */
    explicit ExponentialFilter(double smoothingFactor);

    /**
     * @brief Update filter with new measurement
     * 
     * @param newValue New measurement to filter
     * @return double Filtered value
     */
    double update(double newValue);

    /**
     * @brief Reset filter state
     * 
     * Clears the filter's memory and requires reinitialization
     * with the next update.
     */
    void reset();
};

#endif // EXPONENTIAL_FILTER_H