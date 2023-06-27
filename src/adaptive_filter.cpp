#include "../include/serial_connect/adaptive_filter.h"
#include <cmath>

AdaptiveFilter::AdaptiveFilter(int bufferSize, double threshold)
    : bufferSize(bufferSize), threshold(threshold) {}

double AdaptiveFilter::filter(double value) {
    buffer.push_back(value);

    if (buffer.size() > bufferSize) {
        buffer.pop_front();
    }

    if (isOutlier(value)) {
        double average = calculateAverage();
        return average;
    } else {
        return value;
    }
}

bool AdaptiveFilter::isOutlier(double value) const {
    if (buffer.size() < 4) {
        return false;
    }

    double lastValue = buffer[buffer.size() - 2];
    double secondLastValue = buffer[buffer.size() - 3];
    double thirdLastValue = buffer[buffer.size() - 4];

    double diff = std::abs(value - lastValue);
    double prevDiff = std::abs(lastValue - secondLastValue);
    double secondDiff = std::abs(secondLastValue - thirdLastValue);

    std::cout<<"diff:"<<diff<<" "<<prevDiff<<std::endl;
    if (diff > threshold && diff > prevDiff *2 && diff > secondDiff *2 && value > 300) {
        std::cout<<diff<<prevDiff<<std::endl;
        return true;
    }

    return false;
}

double AdaptiveFilter::calculateAverage() const {
    double sum = 0.0;
    for (double value : buffer) {
        sum += value;
    }
    return sum / buffer.size();
}
