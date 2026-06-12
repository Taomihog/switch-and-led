#include <cstdint>
#include <string>
#include <vector>
#include <assert.h>
#include <unordered_set>
#include <algorithm>
#include <string_view>
#include <functional>

uint64_t hash_n_uint64(const uint64_t* data, size_t n) {
    // 64-bit FNV-1a offset basis
    uint64_t hash = 0xcbf29ce484222325ULL; 
    // 64-bit FNV-1a prime
    const uint64_t fnv_prime = 0x100000001b3ULL; 

    for (size_t i = 0; i < n; ++i) {
        hash ^= data[i];
        hash *= fnv_prime;
    }
    
    // Final avalanche step to mix the upper and lower bits
    hash ^= (hash >> 33);
    hash *= 0xff51afd7ed558ccdULL;
    hash ^= (hash >> 33);
    
    return hash;
}

template<typename T>
std::string mask2string(T bitmask, int length) {
    return std::bitset<sizeof(T) * 8>(bitmask).to_string().substr(sizeof(T) * 8 - length);
}

//A matrix with ncol cols, remove arbitrary n_remove cols, to keep max unique rows. Return number of unqiue rows left and best_removal_masks
static std::pair<int, std::vector<uint64_t>> findBestColumnRemoval(std::vector<uint64_t> matrix, int ncol, int n_remove) {
    assert(n_remove >= 0 && n_remove <= ncol);

    std::sort(matrix.begin(), matrix.end());
    auto new_end = std::unique(matrix.begin(), matrix.end()); 
    matrix.erase(new_end, matrix.end());

    if (n_remove == 0) {
        return { (int)matrix.size(), {0ULL} };
    }
    int max_unique_rows = 0;
    std::vector<uint64_t> best_removal_masks {};
    uint64_t removal_mask = (1ULL << n_remove) - 1;
    const uint64_t limit = 1ULL << ncol;

    // Gosper's hack to iterate through all combinations of n_remove columns to remove
    while (removal_mask < limit) {
        uint64_t keep_mask = (~removal_mask) & (limit - 1);
        std::unordered_set<uint64_t> unique_rows;
        for (const auto& row : matrix) {
            unique_rows.insert(row & keep_mask);
        }
        if (unique_rows.size() >= (size_t)max_unique_rows) {
            if (unique_rows.size() > (size_t)max_unique_rows) {
                max_unique_rows = unique_rows.size();
                best_removal_masks.clear();
            }
            best_removal_masks.push_back(removal_mask);
        }
        // Gosper's hack to get the next combination of n_remove bits
        uint64_t n_remove = removal_mask & -removal_mask;
        uint64_t r = removal_mask + n_remove;
        removal_mask = (((r ^ removal_mask) >> 2) / n_remove) | r;
    }
    return { max_unique_rows, best_removal_masks };
}

// Calculate Hamming distance between two reachables (number of differing bits up to N bits)
uint8_t reachable_distance(uint64_t r1, uint64_t r2, int N) {
    uint64_t mask = (N >= 64) ? ~0ULL : ((1ULL << N) - 1);
    return std::popcount((r1 ^ r2) & mask);
}