// created by xiang on 2024-6-16.
// generate a network of double pole double throw switches with 1 INPUT and M outputs. 
#pragma once
#include <vector>
#include <numeric>
#include <random>
#include <assert.h>
#include <set>
#include <map>
#include <bitset>
#include <iostream>
#include <array>
#include <string>
#include <sstream>
#include <unordered_set>
#include <bit>

#include "DSU_uint8.hpp"

#define DEBUG 1
// define a macro to run function only in debug mode
#define DEBUG_RUN(func) do { if (DEBUG) { func; } } while(0)


/*
Consider this simulation:
We have a power INPUT go into a blackbox, the blockbox has N switches and M output. The switches are double pole double throw.
We generate vertices of pins, the vertices size is an INPUT, 
*/
class SwitchBlackbox {
public:
    enum PoleType { Pole1 = 0, Pole2 = 1 };
    enum PinType { Pin1 = 0, Pin2 = 1, Disconnected = 2 }; 
    static constexpr uint8_t INPUT = 2 << 6 | 3 << 4; 
    static constexpr uint8_t SPECIAL = 3 << 6 | 3 << 4; // special won't belong to pool or wall_pool (pool_size will be less than 16*3+1, so wall_pool size < 16*3)
    using v2p_t = std::array<std::pair<uint8_t, std::array<uint8_t, 0xFF>>, 16>; // vertex index -> a list of pins
    using p2v_t = std::array<uint8_t, 0xFF>; // switch pin to vertex index
    using rsa_t = std::vector<std::pair<uint64_t, uint16_t>>; //reachable_switch_array type

    static constexpr size_t HISTO_SIZE = 100;
    struct statistics {
        size_t total_generate_counter;
        size_t total_check_pass_counter;
        // histogram
        std::array<size_t, HISTO_SIZE> hist_max_reachable_n;
        std::array<size_t, HISTO_SIZE> hist_n_never_reached; // = V - (std::bitset<64>(all_reachable_bitset)).count(); 
        std::array<size_t, HISTO_SIZE> hist_n_unique_configs1; // = static_cast<uint8_t>(reachable_switch_array.size());
        std::array<size_t, HISTO_SIZE> hist_n_unique_configs2; // = static_cast<uint8_t>(reachable_switch_array.size());
        std::array<size_t, HISTO_SIZE> hist_n_unique_configs3; // = static_cast<uint8_t>(reachable_switch_array.size());
        // arrays
        std::vector<uint8_t>  arr_max_reachable_n;
        std::vector<uint8_t>  arr_n_never_reached;
        std::vector<uint16_t> arr_n_unique_configs1;
        std::vector<uint16_t> arr_n_unique_configs2;
        std::vector<uint16_t> arr_n_unique_configs3;
    } stats;

    statistics get_stats_then_reset() {
        statistics stats_out = stats;
        stats = {};
        return stats_out;
    }

    std::string get_readable_config() {
        std::stringstream ss;
        // print all pins in pool
        auto pin2string = [](uint8_t pin) -> std::string {
            if (pin == INPUT) return "Inp";
            auto [poleType, pinType, idx] = decode_pin(pin);
            char poleChar = (poleType == Pole1) ? 'A' : 'B';
            char pinChar = (pinType == Pin1) ? 'a' : (pinType == Pin2) ? 'b' : 'c';
            return std::to_string(idx) + poleChar + pinChar;
        };
        for (int i = 0; i < pool.size(); ++i) {
            uint8_t pin = pool[i];
            ss << pin2string(pin) << " ";
            if (i+ 1 < pool.size()) {
                if (pos2vertex[i] != pos2vertex[i + 1]) ss << "| ";
            }
        }
        ss << std::endl;
        return ss.str();
    }

    SwitchBlackbox(uint8_t n_switches, uint8_t n_outputs, uint8_t n_vertices):
        N(n_switches), M(n_outputs), V(n_vertices), rng(std::random_device{}()), pool_size(static_cast<uint8_t>(1 + 6 * n_switches)), 
        pool(), wall_pool(), stats{}
    {
        assert(N < 16 && M < 16 && V < 64); // Ensure component counts are within bounds, V < 64 because reachable is a uint64_t mask 
        for(int i = 0; i < N; ++i) {
            for (auto pole : {Pole1, Pole2}) {
                for (auto pinType : {Pin1, Pin2, Disconnected}) {
                    pool.push_back(encode_pin(pole, pinType, i));
                }
            }
        }
        pool.push_back(INPUT); // power INPUT is connected to Pole1 by default
        assert(pool.size() == pool_size);        // possible wall positions for 1 to pool_size-1
        for(int i = 1; i < pool_size; ++i) {
            wall_pool.push_back(i);
        }
        assert(wall_pool.size() == pool_size - 1);
    }

    void generate() {
        // step 1. generate circuit
        std::vector<uint8_t> vertex_sizes(V); 
        std::vector<uint8_t> parents(pool_size); // DSU parent array for connectivity simulation
        p2v_t p2pos; // pins to positions in pool
        p2v_t p2v; // pins to vertices
        uint8_t input_pos; // position of the power INPUT pin in the pool
        uint8_t input_vertex; // vertex index of the power INPUT pin, used to quickly check if an output is powered in the simulation
        while (true) {
            ++stats.total_generate_counter;
            update_pools(true);
            // build position -> vertex mapping while keeping parents as DSU start indices
            pos2vertex.assign(pool_size, 0);
            for (int i = 0; i < V; ++i) {
                int begin = (i == 0) ? 0 : wall_pool[i - 1];
                int end = (i == V - 1) ? pool_size : wall_pool[i];
                uint8_t size = static_cast<uint8_t>(end - begin);
                vertex_sizes[i] = size;
                for (int j = begin; j < end; ++j) {
                    parents[j] = begin; // DSU parent points to segment start
                    pos2vertex[j] = static_cast<uint8_t>(i); // map pool position to vertex index
                }
            }

            // one-pass to fill out input_pos, input_vertex, p2pos and p2v for quick constraint evaluation and simulation later
            for (int i = 0; i < pool.size(); ++i) {
                uint8_t pin = pool[i];
                if (pin == INPUT) {
                    input_pos = static_cast<uint8_t>(i);
                    input_vertex = pos2vertex[i];
                    continue;
                }
                p2pos[pin] = static_cast<uint8_t>(i);
                p2v[pin] = pos2vertex[i];
            }
            if(validate_constraints(N, vertex_sizes, p2v, input_vertex)) break;
        }

        // step 2. generate_reachable_switch_array
        // the circuit is determined by pool and wall positions (or parents, or p2pos), now we want to calculate what vertices will be reachable at different switch configs
        reachable_switch_array.clear();
        for(uint16_t bitmask = 0; bitmask < (1 << N); ++bitmask) {
            DSU_uint8 dsu(std::vector<uint8_t>{parents});
            for(int i = 0; i < N; ++i) {
                dsu.unite(p2pos[encode_pin(Pole1, Disconnected, i)], ((bitmask >> i) & 1) == 0 ? p2pos[encode_pin(Pole1, Pin1, i)] : p2pos[encode_pin(Pole1, Pin2, i)]);
                dsu.unite(p2pos[encode_pin(Pole2, Disconnected, i)], ((bitmask >> i) & 1) == 0 ? p2pos[encode_pin(Pole2, Pin1, i)] : p2pos[encode_pin(Pole2, Pin2, i)]);
            }

            // each vertex can be an output
            // the original vertices united with the INPUT vertex are set to 1, other vertices are 0
            const uint8_t input_root = dsu.find(input_pos);
            uint64_t reachable_bitset = 0ULL;
            for (int i = 0; i < pool.size(); ++i) {
                if (dsu.find(i) == input_root) {
                    auto pin = pool[i];
                    if (pin == INPUT) continue;
                    uint8_t v = p2v[pin];
                    assert(v < V); // guard against invalid vertex index
                    reachable_bitset |= 1ULL << v;
                }
            }
            reachable_switch_array.emplace_back(reachable_bitset, bitmask);
        }
    }

    bool unique_vertices_no_less_than_m() {
        // how many unique reachable pattern
        std::sort(reachable_switch_array.begin(), reachable_switch_array.end(), [](const auto& a, const auto& b) {
            if (a.first == b.first) return a.second < b.second;
            return a.first < b.first;
        });
        auto new_end = std::unique(reachable_switch_array.begin(), reachable_switch_array.end(), [](const auto& a, const auto& b) {
            return a.first == b.first;
        }); 
        reachable_switch_array.erase(new_end, reachable_switch_array.end());

        uint64_t all_reachable_bitset = 0ULL; // union of outputs reachable by any switch configuration
        uint8_t  max_reachable_n = 0;
        uint16_t max_reachable_bitmask = 0;
        uint64_t max_reachable_bitset = 0;
        for(const auto&[r,m] : reachable_switch_array) {
            all_reachable_bitset |= r;
            uint8_t cnt = static_cast<uint8_t>((std::bitset<64>(r)).count());
            if (cnt > max_reachable_n) {
                max_reachable_n = cnt;
                max_reachable_bitmask = m;
                max_reachable_bitset = r;
            }
        }
        const uint8_t n_never_reached = V - (std::bitset<64>(all_reachable_bitset)).count(); 
        const uint16_t n_unique_configs = static_cast<uint16_t>(reachable_switch_array.size());//bitmask size < 1<<16        //print

        std::cout << "============================================================Unique Vertices >= M, Config=============================================================";
        std::cout << std::endl;
        std::cout << "#switch N: " << (int)N << std::endl;
        std::cout << "#output M: " << (int)M << std::endl;
        std::cout << "#vertex V: " << (int)V << std::endl;
        std::cout << "pool size: " << (int)pool_size << std::endl << std::endl;
        std::cout << "Total tries: " << stats.total_generate_counter << std::endl;
        std::cout << "Unique configurations: " << (int)n_unique_configs << " (" << (1<<N - (int)n_unique_configs) << " removed)" << std::endl;
        std::cout << "All reachable bitset: " << mask2string(all_reachable_bitset, V) << std::endl;
        std::cout << "Never reached outputs: " << (int)n_never_reached << std::endl;
        std::cout << "Max reachable outputs: " << (int)max_reachable_n << std::endl;
        std::cout << "bitmask|reachable: " << mask2string(max_reachable_bitmask, N) << "|" << mask2string(max_reachable_bitset, V) << std::endl;
        std::cout << "vertices_max_reachable_pattern_can_reach size (for heuristic selection): ";
        for(auto&[reachable, bitmask] : reachable_switch_array) {
            std::cout << "bitmask|reachable: " << mask2string(bitmask, N) << "|" << mask2string(reachable, V) << std::endl;
        }
        std::cout << "Vertex and pins \n";
        // print all pins in pool
        auto pin2string = [](uint8_t pin) -> std::string {
            if (pin == INPUT) return "Inp";
            auto [poleType, pinType, idx] = decode_pin(pin);
            char poleChar = (poleType == Pole1) ? 'A' : 'B';
            char pinChar = (pinType == Pin1) ? 'a' : (pinType == Pin2) ? 'b' : 'c';
            return std::to_string(idx) + poleChar + pinChar;
        };
        std::cout << "pins in pool:      ";
        for (int i = 0; i < pool.size(); ++i) {
            uint8_t pin = pool[i];
            std::cout << pin2string(pin) << " ";
        }
        std::cout << std::endl;
        std::cout << "pins in vertices: ";
        for (int i = 0; i < pool.size(); ++i) {
            printf(" %2d ", pos2vertex[i]);
        }
        std::cout << std::endl;
        std::cout << "test: " << get_readable_config();
        std::cout << std::endl;
        std::cout << "=====================================================================================================================================================" << std::endl << std::endl;

        ++stats.total_check_pass_counter;
        ++stats.hist_max_reachable_n[max_reachable_n];                  stats.arr_max_reachable_n.push_back(max_reachable_n);
        ++stats.hist_n_never_reached[n_never_reached];                  stats.arr_n_never_reached.push_back(n_never_reached);
        ++stats.hist_n_unique_configs1[n_unique_configs];               stats.arr_n_unique_configs1.push_back(n_unique_configs);
        return max_reachable_n >= M;
    }

    void best_choice_of_m_vertices() {
        const auto& ori_arr = reachable_switch_array;
        int max_unique_rows = 0;
        std::vector<uint64_t> best_removal_masks {};
        for(auto[reachable, bitmask] : ori_arr) {
            const int nbit1 = std::popcount(reachable);
            if (nbit1 < M) continue;

            std::cout << "mask: " << mask2string(bitmask, N) << std::endl;

            // bit 1 positions
            std::vector<int> positions;
            while (reachable > 0) {
                // Find the index of the lowest set bit (0-indexed)
                int pos = std::countr_zero(reachable);
                positions.push_back(pos);
                
                // Clear the lowest set bit using Brian Kernighan's trick
                reachable &= (reachable - 1); 
            }
            assert(nbit1 == positions.size());

            std::cout << "Shrink the size of reachable from V to nbit1:" << std::endl;
            rsa_t new_arr{}; 
            for(const auto&[reachable, bitmask] : ori_arr) {
                uint64_t new_reachable = 0;
                int bit = 0;
                for (int v : positions) {
                    new_reachable |= ((reachable >> v) & 1ULL) << bit;
                    bit++;
                }
                new_arr.emplace_back(new_reachable, bitmask);
                std::cout << "bitmask|reachable: " << mask2string(bitmask, N) << "|" << mask2string(reachable, nbit1) << std::endl;
            }

            std::sort(new_arr.begin(), new_arr.end(), [](const auto& a, const auto& b) {
                if (a.first == b.first) return a.second < b.second;
                return a.first < b.first;
            });
            auto new_end = std::unique(new_arr.begin(), new_arr.end(), [](const auto& a, const auto& b) {
                return a.first == b.first;
            }); 
            new_arr.erase(new_end, new_arr.end());
            
            std::cout << "\nAfter removing duplicates, unique configurations: " << (int)new_arr.size() << std::endl;
            std::cout << std::endl;
            for(const auto&[reachable, bitmask] : new_arr) {
                std::cout << "bitmask|reachable: " << mask2string(bitmask, N) << "|" << mask2string(reachable, nbit1) << std::endl;
            }

            auto [nrow, masks] = findBestColumnRemoval(new_arr, nbit1, nbit1 - M);
            // conver the best_removal_masks to the real best_removal masks before shrinking
            for(uint64_t &mask: masks) {
                auto old = mask;
                old = ~old; // best keep mask;
                assert(std::popcount(old) == M);
                mask = 0;
                while (old > 0) {
                    // Find the index of the lowest set bit (0-indexed)
                    int pos = std::countr_zero(old);
                    mask |= 1ULL << (positions[pos]);
                    
                    // Clear the lowest set bit using Brian Kernighan's trick
                    old &= (old - 1); 
                }
                assert(std::popcount(mask) == M);
            }
            if (nrow >= max_unique_rows) {
                if (nrow > max_unique_rows) {
                    max_unique_rows = nrow;
                    best_removal_masks.clear();
                }
                best_removal_masks.insert(best_removal_masks.end(), masks.begin(), masks.end());
            }
        }
        // update the vertices_max_reachable_pattern_can_reach
        std::cout << "\nAfter selecting the best M output from the max reachable bitset, unique configurations: " << max_unique_rows << std::endl;
        std::cout << "Best removal mask(s): " << std::endl;
        for (auto mask : best_removal_masks) {
            std::cout << mask2string(mask, V) << std::endl;
        }
        ++stats.hist_n_unique_configs2[max_unique_rows];          stats.arr_n_unique_configs2.push_back(max_unique_rows);
    }

    void find_max_entropy_unique_ros() {};

private:
    // 1. constants
    const uint8_t N; // number of switches
    const uint8_t M; // number of outputs
    const uint8_t V; // number of vertices
    std::mt19937 rng; // random number generator
    const uint8_t pool_size; // total number of pins, should be 2 for power + 2*M for LEDs + 3*N for switches

    // 2. semi constants, only allow shuffle at the beginning of each generation
    std::vector<uint8_t> pool; // pool of pins to be shuffled and assigned to vertices
    std::vector<uint8_t> wall_pool; 

    // 3. data from generate(), they are constant outside of generate() but need to be stored as member variables for easy access in both constraint validation and simulation
    rsa_t reachable_switch_array; // map reachable outputs to switch config
    std::vector<uint8_t> pos2vertex; // pool position -> vertex index mapping

    uint8_t best_unique_config = 0;

    // uint8_t is enough, the simulation will for sure use less than 16 switches and 16 LEDs, and we can encode the component type and pin type in the upper bits.
    static uint8_t encode_pin(PoleType poleType, PinType pinType, uint8_t idx) {
        assert (idx < 16);
        return (poleType << 6) | (pinType << 4) | idx;
    }

    static std::tuple<PoleType, PinType, uint8_t> decode_pin(uint8_t pin) {
        PoleType poleType = static_cast<PoleType>(pin >> 6);
        PinType pinType = static_cast<PinType>((pin >> 4) & 0x3);
        uint8_t idx = pin & 0xF;
        return std::make_tuple(poleType, pinType, idx);
    }

    void update_pools(bool method) {
        if (method) {
            std::shuffle(pool.begin(), pool.end(), rng);
            std::shuffle(wall_pool.begin() , wall_pool.end(), rng);
            std::sort(wall_pool.begin(), wall_pool.begin() + V - 1); // Sort wall_pool[0] through wall_pool[V-2] (the V-1 internal walls we use)
        } else {
            // TODO: load from a txt file
        }
    }

    static bool validate_constraints(const int N, const std::vector<uint8_t>& vertex_sizes, const p2v_t& p2v, const uint8_t input_vertex) {
        // power vertex size must > 1
        if (vertex_sizes[input_vertex] <= 1) {
            return false;
        }
        // for each pole of each switch, 2 pins cannot be in the same vertex
        for (int i = 0; i < N; ++i) {
            if (p2v[encode_pin(Pole1, Pin1, i)]         == p2v[encode_pin(Pole1, Pin2, i)] || 
                p2v[encode_pin(Pole1, Disconnected, i)] == p2v[encode_pin(Pole1, Pin1, i)] || 
                p2v[encode_pin(Pole1, Disconnected, i)] == p2v[encode_pin(Pole1, Pin2, i)] ||
                p2v[encode_pin(Pole2, Pin1, i)]         == p2v[encode_pin(Pole2, Pin2, i)] ||
                p2v[encode_pin(Pole2, Disconnected, i)] == p2v[encode_pin(Pole2, Pin1, i)] ||
                p2v[encode_pin(Pole2, Disconnected, i)] == p2v[encode_pin(Pole2, Pin2, i)]) {
                return false;
            }
        }
        // for each pole of each switch, the vertices of pin1 and pin2 can be of any sizes, but the vertices of pin1 and disconnected cannot be both size 1, same for pin2 and disconnected, 
        // otherwise the switch is not really a switch because the disconnected pin is basically not connected to anything else and won't affect the connectivity at all.
        for (int i = 0; i < N; ++i) {
            if ((vertex_sizes[p2v[encode_pin(Pole1, Pin1, i)]] == 1 && vertex_sizes[p2v[encode_pin(Pole1, Disconnected, i)]] == 1) ||
                (vertex_sizes[p2v[encode_pin(Pole1, Pin2, i)]] == 1 && vertex_sizes[p2v[encode_pin(Pole1, Disconnected, i)]] == 1) ||
                (vertex_sizes[p2v[encode_pin(Pole2, Pin1, i)]] == 1 && vertex_sizes[p2v[encode_pin(Pole2, Disconnected, i)]] == 1) ||
                (vertex_sizes[p2v[encode_pin(Pole2, Pin2, i)]] == 1 && vertex_sizes[p2v[encode_pin(Pole2, Disconnected, i)]] == 1)) {
                return false;
            }
        }
        return true; 
    }

    template<typename T>
    static std::string mask2string(T bitmask, int length) {
        return std::bitset<sizeof(T) * 8>(bitmask).to_string().substr(sizeof(T) * 8 - length);
    }

    static std::pair<int, std::vector<uint64_t>> findBestColumnRemoval(const rsa_t& rsa, int M, int C) {
        int N = rsa.size();
        int max_unique_rows = 0;
        std::vector<uint64_t> best_removal_masks {};
        uint64_t removal_mask = (1ULL << C) - 1;
        const uint64_t limit = 1ULL << M;
        
        // Gosper's hack to iterate through all combinations of C columns to remove
        while (removal_mask < limit) {
            uint64_t keep_mask = (~removal_mask) & (limit - 1);
            std::unordered_set<uint64_t> unique_rows;
            for (const auto& [row, bitmask] : rsa) {
                unique_rows.insert(row & keep_mask);
            }
            if (unique_rows.size() >= max_unique_rows) {
                if (unique_rows.size() > max_unique_rows) {
                    max_unique_rows = unique_rows.size();
                    best_removal_masks.clear();
                }
                best_removal_masks.push_back(removal_mask);
            }
            // Gosper's hack to get the next combination of C bits
            uint64_t c = removal_mask & -removal_mask;
            uint64_t r = removal_mask + c;
            removal_mask = (((r ^ removal_mask) >> 2) / c) | r;
        }
        return { max_unique_rows, best_removal_masks };
    }

    static std::vector<uint8_t> serialize_data(const SwitchBlackbox& bb) {
        // serialize the data so I can save it for analysis
        // N,M,V,pool_size,pool[0],...,pool[pool_size - 1],wall_pool.size(),wall_pool[0],...,wall_pool[wall_pool.size() - 2]
        std::vector<uint8_t> serialized_data;
        serialized_data.push_back(bb.N);
        serialized_data.push_back(bb.M);
        serialized_data.push_back(bb.V);
        serialized_data.push_back(bb.pool_size);
        for(int i = 0; i < serialized_data.back(); ++i) {
            serialized_data.push_back(bb.pool[i]);
        }
        serialized_data.push_back(bb.wall_pool.size());
        for(int i = 0; i < serialized_data.back(); ++i) {
            serialized_data.push_back(bb.wall_pool[i]);
        }
        serialized_data.push_back(SPECIAL);
        //some metrics

        return serialized_data;
    }

};
