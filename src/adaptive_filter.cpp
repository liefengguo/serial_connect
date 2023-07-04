#include "../include/serial_connect/adaptive_filter.h"
#include <cmath>

AdaptiveFilter::AdaptiveFilter(int bufferSize, int threshold)
    : bufferSize(bufferSize), threshold(threshold) {}

int AdaptiveFilter::filter(int value) {
    buffer.push_back(value);

    if (buffer.size() > bufferSize) {
        buffer.pop_front();
    }

    if (isOutlier(value)) {
        int average = calculateAverage();
        return average;
    } else {
        return value;
    }
}
// int AdaptiveFilter::filter(int value) {
//     buffer.push_back(value);

//     if (buffer.size() > bufferSize) {
//         buffer.pop_front();
//     }

//     if (isOutlier(value)) {
//         int average = calculateAverage();
//         return average;
//     } else {
//         return value;
//     }
// }
bool AdaptiveFilter::isOutlier(int value) const {
    if (buffer.size() < 4) {
        return false;
    }

    int lastValue = buffer[buffer.size() - 2];
    int secondLastValue = buffer[buffer.size() - 3];
    int thirdLastValue = buffer[buffer.size() - 4];

    int diff = std::abs(value - lastValue);
    int prevDiff = std::abs(lastValue - secondLastValue);
    int secondDiff = std::abs(secondLastValue - thirdLastValue);

    std::cout<<"diff:"<<diff<<" "<<prevDiff<<std::endl;
    if (diff > threshold && diff > prevDiff *2 && diff > secondDiff *2 && value > 300) {
        std::cout<<diff<<prevDiff<<std::endl;
        return true;
    }

    return false;
}

int AdaptiveFilter::calculateAverage() const {
    int sum = 0.0;
    for (int value : buffer) {
        sum += value;
    }
    return sum / buffer.size();
}
