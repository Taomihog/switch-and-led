#pragma once
#include <vector>
#include <cassert>
#include <utility>
#include <cstddef>
#include <algorithm>
#include <bit>

namespace Matrix {

template <typename T>
void keep_uniques(std::vector<T>& matrix) {
    std::sort(matrix.begin(), matrix.end());
    matrix.erase(std::unique(matrix.begin(), matrix.end()), matrix.end());
}

template <typename T>
inline size_t uniqueness_sorted(std::vector<T>& vec) {
    size_t duplicate_count = 0;
    for (int i = 0; i < vec.size() - 1; i++) {
        if (arr[i] == arr[i + 1]) {
            ++duplicate_count;
        }
    }
    return vec.size() - duplicate_count;
}

template <typename T>
int stdev(const std::vector<T>& arr) {
    std::vector<int> bitCounts(64, 0);
    for (const auto& r : arr) {
        for (int i = 0; i < 64; ++i) {
            if ((r >> i) & 1) {
                bitCounts[i]++;
            }
        }
    }
    int sum = 0;
    int square_sum = 0;
    for (const auto c : bitCounts) {
        sum += c;
        square_sum += c * c;
    }
    return 64 * square_sum - sum * sum ;
}

template <typename T>
int n1s(const std::vector<T>& arr) { // number of bit1s
    int n1s = 0;
    for (const auto& r : arr) {
        n1s += std::popcount(r);
    }
    return n1s;
}

// (pack_matrix and unpack_mask functions remain exactly the same as before)
template<typename T>
static std::vector<T> pack_matrix(const std::vector<T>& matrix, T allowed_mask) {
    std::vector<T> packed_matrix;
    packed_matrix.reserve(matrix.size());
    for (const auto& row : matrix) {
        T packed_row = 0;
        T temp_allowed = allowed_mask;
        int target_bit = 0;
        while (temp_allowed != 0) {
            T lowest_bit = temp_allowed & -temp_allowed;
            if (row & lowest_bit) {
                packed_row |= (static_cast<T>(1) << target_bit);
            }
            target_bit++;
            temp_allowed &= (temp_allowed - 1);
        }
        packed_matrix.push_back(packed_row);
    }
    return packed_matrix;
}

template<typename T>
T unpack_mask(T packed_mask, T allowed_mask) {
    T result = 0;
    T temp_allowed = allowed_mask;
    int source_bit = 0;
    while (temp_allowed != 0) {
        T lowest_bit = temp_allowed & -temp_allowed;
        if ((packed_mask >> source_bit) & 1) {
            result |= lowest_bit;
        }
        source_bit++;
        temp_allowed &= (temp_allowed - 1);
    }
    return result;
}

template<typename T>
static std::pair<int, std::vector<T>> findBestColumnRetention(
    const std::vector<T>& matrix, 
    T allowed_mask, 
    int n_keep
) {
    int total_allowed_cols = std::popcount(static_cast<uint64_t>(allowed_mask));
    assert(n_keep >= 0 && n_keep <= total_allowed_cols);

    if (n_keep == 0) {
        return { 1, {0} };
    }

    std::vector<T> packed_matrix = pack_matrix(matrix, allowed_mask);

    int max_unique_rows = 0;
    std::vector<T> best_packed_keep_masks{};

    // Optimization: Pre-allocate a scratchpad vector to avoid repeated memory allocations
    std::vector<T> scratchpad;
    scratchpad.reserve(packed_matrix.size());

    T keep_mask = (static_cast<T>(1) << n_keep) - 1;
    const T packed_limit = static_cast<T>(1) << total_allowed_cols;

    while (keep_mask < packed_limit) {
        // Step 1: Reuse memory and quickly mask rows
        scratchpad.clear();
        for (const auto& row : packed_matrix) {
            scratchpad.push_back(row & keep_mask);
        }

        // Step 2: Sort elements to bring identical items together
        std::sort(scratchpad.begin(), scratchpad.end());

        // Step 3: Use std::unique to shift duplicates to the end, then count unique items
        auto unique_end = std::unique(scratchpad.begin(), scratchpad.end());
        int unique_count = std::distance(scratchpad.begin(), unique_end);

        // Step 4: Evaluate results
        if (unique_count >= max_unique_rows) {
            if (unique_count > max_unique_rows) {
                max_unique_rows = unique_count;
                best_packed_keep_masks.clear();
            }
            best_packed_keep_masks.push_back(keep_mask);
        }

        // Gosper's hack transition
        T lowest_bit = keep_mask & -keep_mask;
        T r = keep_mask + lowest_bit;
        keep_mask = (((r ^ keep_mask) >> 2) / lowest_bit) | r;
    }

    // Unpack winning configurations back to their spatial masks
    std::vector<T> best_keep_masks;
    best_keep_masks.reserve(best_packed_keep_masks.size());
    for (const auto& packed_mask : best_packed_keep_masks) {
        best_keep_masks.push_back(unpack_mask(packed_mask, allowed_mask));
    }

    return { max_unique_rows, best_keep_masks };
}

}