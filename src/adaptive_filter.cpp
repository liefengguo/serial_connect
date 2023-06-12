#include "../include/serial_connect/adaptive_filter.h"
#include <algorithm>

AdaptiveFilter::AdaptiveFilter(size_t size, double multiplier)
    : bufferSize(size), thresholdMultiplier(multiplier), sum(0.0) {
    buffer.reserve(bufferSize);
}

double AdaptiveFilter::filter(double value) {
    if (isOutlier(value)) {
        return buffer.back();  // 返回缓冲区中最后一个值
    }

    buffer.push_back(value);
    sum += value;

    if (buffer.size() > bufferSize) {
        sum -= buffer.front();
        buffer.erase(buffer.begin());
    }

    return sum / buffer.size();
}

bool AdaptiveFilter::isOutlier(double value) {
    if (buffer.size() < bufferSize) {
        return false;
    }

    double mean = sum / buffer.size();
    double deviation = std::abs(value - mean);
    return deviation > thresholdMultiplier * deviation;
}
