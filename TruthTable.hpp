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

// Assuming tt_t struct is defined elsewhere in your namespace

template<typename T> 
std::pair<T, std::vector<std::vector<size_t>>> relevant_bits(const tt_t<T>& tt) {
    constexpr uint64_t ONE = 1; // Prevent overflow on architectures with 32-bit size_t
    
    const int N = tt.N;
    const int V = tt.V;

    std::vector<size_t> relevance_row(N, 0); 
    std::vector<std::vector<size_t>> relevance(V, relevance_row);

    // Phase 1: Compute total hypercube edge transitions
    for (size_t x = 0; x < (ONE << N); ++x) { 
        for (int i = 0; i < N; ++i) {
            if (!(x & (ONE << i))) { 
                size_t neighbor = x | (ONE << i);
                
                T diff = tt.outputs[x] ^ tt.outputs[neighbor];
                diff &= ((static_cast<T>(1) << V) - 1);
                if (diff != 0) {
                    T temp_diff = diff;
                    while (temp_diff != 0) {
                        int j = std::countr_zero(temp_diff); 
                        
                        ++relevance[j][i];
                        
                        temp_diff &= (temp_diff - 1); 
                    }
                }
            }
        }
    }

    // Phase 2: Construct the mask for bits related to ALL input bits
    T mask_all = 0;
    for (int j = 0; j < V; ++j) {
        bool related_to_all = true;
        
        for (int i = 0; i < N; ++i) {
            if (relevance[j][i] == 0) {
                related_to_all = false; // Output j does not depend on input i
                break;
            }
        }
        
        if (related_to_all) {
            mask_all |= (static_cast<T>(1) << j);
        }
    }

    return {mask_all, relevance}; 
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
    for (size_t i = 0; i < 1ULL << tt.N; ++i) {
        for (size_t j = i + 1; j < 1ULL << tt.N; ++j) {
            if (std::popcount(i ^ j) == dist) {
                total_distance += reachable_distance(tt.outputs[i], tt.outputs[j], tt.V);
            }
        }
    }
    return total_distance;
}

}