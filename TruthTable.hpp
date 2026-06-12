#pragma once
#include <vector>
#include <cstdint>
#include <cstddef>
#include <bit>
#include <cassert>
#include <utility>
#include <unordered_set>


namespace TruthTable {

template<typename T> 
struct tt_t {
    tt_t(int N_in, int V_in, const std::vector<T>& outputs_in)
        : N(N_in), V(V_in), outputs(outputs_in.begin(), outputs_in.end()) 
    {
        assert(outputs.size() == (1ULL << N));
    }

    const int N;
    const int V;
    const std::vector<T> outputs; 
};

template<typename T> 
std::pair<T, std::vector<std::vector<int>>> relevant_bits(const tt_t<T>& tt) {
    T mask = 0;
    constexpr size_t ONE = 1; 
    
    const int N = tt.N;
    const int V = tt.V;

    std::vector<int> relevance_row(N, 0); 
    std::vector<std::vector<int>> relevance(V, relevance_row);

    for (size_t x = 0; x < (ONE << N); ++x) { 
        for (int i = 0; i < N; ++i) {
            if (!(x & (ONE << i))) { 
                size_t neighbor = x | (ONE << i);
                
                T diff = tt.outputs[x] ^ tt.outputs[neighbor];
                if (diff != 0) {
                    mask |= diff; 
                    
                    // Bit Manipulation Optimization
                    T temp_diff = diff;
                    while (temp_diff != 0) {
                        // Instantly find the index of the lowest set bit
                        int j = std::countr_zero(temp_diff); 
                        
                        ++relevance[j][i];
                        
                        // Clear the lowest set bit (BLSR instruction equivalent)
                        temp_diff &= (temp_diff - 1); 
                    }
                }
            }
        }
    }
    return {mask, relevance};
}

// Calculate Hamming distance between two outputs (number of differing bits up to V bits)
template<typename T>
int reachable_distance(T r1, T r2, int V) {
    // If V is 64 or more, the mask covers the full 64 bits
    uint64_t mask = (V >= 64) ? ~0ULL : ((1ULL << V) - 1);
    
    // Cast to uint64_t safely to match the mask type for std::popcount
    return std::popcount(static_cast<uint64_t>(r1 ^ r2) & mask);
}

template<typename T>
uint32_t sum_output_distance_for_adjacent_inputs(int dist, const tt_t<T>& tt) {
    uint32_t total_distance = 0;
    constexpr size_t ONE = 1;
    
    const int N = tt.N;
    const int V = tt.V;
    const size_t num_states = ONE << N;

    // Use nested index loops directly instead of an expensive std::map
    for (size_t i = 0; i < num_states; ++i) {
        for (size_t j = i + 1; j < num_states; ++j) {
            // Check if the input states are at the requested Hamming distance
            size_t xor_val = i ^ j;
            if (std::popcount(xor_val) == dist) {
                // Pass tt.V to distance function to limit output bit checking
                total_distance += reachable_distance(tt.outputs[i], tt.outputs[j], V);
            }
        }
    }
    return total_distance;
}

}