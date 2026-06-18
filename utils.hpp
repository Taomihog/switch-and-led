#pragma once
#include <cstdint>
#include <string>
#include <bitset>

namespace utils {
[[nodiscard]] uint64_t hash_n_uint64(const uint64_t* data, size_t n) {
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
[[nodiscard]] std::string mask2string(T bitmask, int length) {
    return std::bitset<sizeof(T) * 8>(bitmask).to_string().substr(sizeof(T) * 8 - length);
}

// deprecated, use Matrix::pack_matrix and Matrix::unpack_mask for similar functions
#if 0
// bit 1 positions
template <typename T>
std::vector<int> positions_of_bit1(T reachable) {
    int nbit1 = std::popcount(reachable);
    std::vector<int> positions;
    while (reachable > 0) {
        // Find the index of the lowest set bit (0-indexed)
        int pos = std::countr_zero(reachable);
        positions.push_back(pos);
        
        // Clear the lowest set bit using Brian Kernighan's trick
        reachable &= (reachable - 1); 
    }
    assert(nbit1 == (int) positions.size() && "positions of bits size incorrect.");
    return positions;
}

template <typename T>
T shrink_by_bitmask_position(const T& old, const std::vector<int>& positions) {
    uint64_t masked = 0;
    int bit = 0;
    for (int v : positions) {
        masked |= ((old >> v) & 1ULL) << bit;
        bit++;
    }
    return masked;
}
#endif

}



