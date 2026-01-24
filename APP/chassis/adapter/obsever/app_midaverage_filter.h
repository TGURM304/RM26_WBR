//
// Created by 15082 on 2026/1/18.
//

#ifndef APP_MIDAVERAGE_FILTER_H
#define APP_MIDAVERAGE_FILTER_H
#include <cstdint>


template <std::size_t N, typename T = int>
class TrimmedMeanFilter {
    static_assert(N >= 3, "N must be >= 3");

public:
    TrimmedMeanFilter() : index_(0), count_(0) {}

    // ① 输入新数据
    void input(T value) {
        buffer_[index_] = value;
        index_ = (index_ + 1) % N;
        if (count_ < N) {
            ++count_;
        }
    }

    // ② 获取滤波结果
    T get() const {
        if (count_ < N) {
            return 0;  // 未就绪时返回 0（或默认值）
        }

        T sum = buffer_[0];
        T min_v = buffer_[0];
        T max_v = buffer_[0];

        for (std::size_t i = 1; i < N; ++i) {
            T v = buffer_[i];
            sum += v;
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }

        return (sum - min_v - max_v) / static_cast<T>(N - 2);
    }

private:
    T buffer_[N];
    std::size_t index_;
    std::size_t count_;
};




#endif //APP_MIDAVERAGE_FILTER_H
