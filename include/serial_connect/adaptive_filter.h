#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <vector>

class AdaptiveFilter {
private:
    std::vector<double> buffer;
    size_t bufferSize;
    double thresholdMultiplier;
    double sum;

public:
    AdaptiveFilter(size_t size, double multiplier);

    double filter(double value);

private:
    bool isOutlier(double value);
};

#endif  // ADAPTIVE_FILTER_H
