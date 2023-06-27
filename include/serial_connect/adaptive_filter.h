#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <deque>
#include <iostream>
class AdaptiveFilter {
public:
    AdaptiveFilter(int bufferSize, double threshold);

    double filter(double value);

private:
    int bufferSize;
    double threshold;
    std::deque<double> buffer;

    bool isOutlier(double value) const;
    double calculateAverage() const;
};

#endif  // ADAPTIVE_FILTER_H
