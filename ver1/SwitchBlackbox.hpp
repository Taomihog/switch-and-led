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
#include "utils.hpp"

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
    bool verbose = false;
    size_t total_generate_counter = 0;

    static constexpr size_t HISTO_SIZE = 100;
    struct Statistics {
        uint8_t N = 0;
        uint8_t M = 0;
        uint8_t V = 0;
        uint8_t pool_size = 0;
        uint64_t hash = 0; 
        uint64_t all_reachable_bitset = 0ULL;
        uint8_t  max_reachable_n = 0;
        uint16_t max_reachable_bitmask = 0;
        uint64_t max_reachable_bitset = 0;
        uint8_t n_never_reached = 0; 
        uint16_t n_unique_configs = 0;
        size_t total_generate_counter = 0;
    };

    std::string get_readable_config(bool single_line = true) const {
        std::stringstream ss;
        // print all pins in pool
        auto pin2string = [](uint8_t pin) -> std::string {
            if (pin == INPUT) return "Inp";
            auto [poleType, pinType, idx] = decode_pin(pin);
            char poleChar = (poleType == Pole1) ? 'A' : 'B';
            char pinChar = (pinType == Pin1) ? 'a' : (pinType == Pin2) ? 'b' : 'c';
            return std::to_string(idx) + poleChar + pinChar;
        };
        if (single_line) {
            
            for (size_t i = 0; i < pool.size(); ++i) {
                uint8_t pin = pool[i];
                ss << pin2string(pin) << " ";
                if (i < pool.size() - 1) {
                    if (pos2vertex[i] != pos2vertex[i + 1]) ss << "| ";
                }
            }
            ss << std::endl;
        } else {
            ss << "pins in pool:      ";
            for (size_t i = 0; i < pool.size(); ++i) {
                uint8_t pin = pool[i];
                ss << pin2string(pin) << " ";
            }
            ss << std::endl;
            ss << "pins in vertices: ";
            for (size_t i = 0; i < pool.size(); ++i) {
                printf(" %2d ", pos2vertex[i]);
            }
        }
        return ss.str();
    }

    const rsa_t& get_reachable_switch_array() const {
        return reachable_switch_array;
    }

    // Sum of distances between reachables for all pairs of bitmasks that differ by exactly dist bit (adjacent in Hamming space)
    uint32_t sum_reachable_distance_for_adjacent_bitmasks(int dist, int N) const {
        std::map<uint16_t, uint64_t> bitmask_to_reachable;
        for (const auto& [reachable, bitmask] : reachable_switch_array) {
            bitmask_to_reachable[bitmask] = reachable;
        }
        
        uint32_t total_distance = 0;
        const auto& bitmasks = bitmask_to_reachable;
        for (auto it1 = bitmasks.begin(); it1 != bitmasks.end(); ++it1) {
            for (auto it2 = std::next(it1); it2 != bitmasks.end(); ++it2) {
                uint16_t xor_val = it1->first ^ it2->first;
                if (std::popcount(static_cast<uint32_t>(xor_val)) == dist) {
                    total_distance += reachable_distance(it1->second, it2->second, N);
                }
            }
        }
        return total_distance;
    }

    int stdev() {
        std::vector<int> bitCounts(64, 0);
        for (const auto& [r, b] : reachable_switch_array) {
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

    int n_1s() {
        int n1s = 0;
        for (const auto& [r, b] : reachable_switch_array) {
            n1s += std::popcount(r);
        }
        return n1s;
    }

    SwitchBlackbox(uint8_t n_switches, uint8_t n_outputs, uint8_t n_vertices):
        N(n_switches), M(n_outputs), V(n_vertices), rng(std::random_device{}()), pool_size(static_cast<uint8_t>(1 + 6 * n_switches)), 
        pool(), wall_pool()
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
        assert(wall_pool.size() == size_t (pool_size - 1));
    }

    void generate(std::string s = "") {
        // step 1. generate circuit
        std::vector<uint8_t> vertex_sizes(V); 
        std::vector<uint8_t> parents(pool_size); // DSU parent array for connectivity simulation
        p2v_t p2pos; // pins to positions in pool
        p2v_t p2v; // pins to vertices
        uint8_t input_pos; // position of the power INPUT pin in the pool
        uint8_t input_vertex; // vertex index of the power INPUT pin, used to quickly check if an output is powered in the simulation
        while (true) {
            ++total_generate_counter;
            if (!s.length()) { // generate a random pool and wall_pool
                std::shuffle(pool.begin(), pool.end(), rng);
                std::shuffle(wall_pool.begin() , wall_pool.end(), rng);
                std::sort(wall_pool.begin(), wall_pool.begin() + V - 1); // Sort wall_pool[0] through wall_pool[V-2] (the V-1 internal walls we use)
            } else { // load pool and wall_pool
                update_pools(s);
            }
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
            for (size_t i = 0; i < pool.size(); ++i) {
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
            for (size_t i = 0; i < pool.size(); ++i) {
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

    Statistics basic_stats() {
        // remove duplicates from rsa
        std::sort(reachable_switch_array.begin(), reachable_switch_array.end(), [](const auto& a, const auto& b) {
            if (a.first == b.first) return a.second < b.second;
            return a.first < b.first;
        });
        auto new_end = std::unique(reachable_switch_array.begin(), reachable_switch_array.end(), [](const auto& a, const auto& b) {
            return a.first == b.first;
        }); 
        reachable_switch_array.erase(new_end, reachable_switch_array.end());
        Statistics stats {};
        stats.N = N;
        stats.M = M;
        stats.V = V;
        stats.pool_size = pool_size;
        uint64_t& hash = stats.hash;
        // 64-bit FNV-1a offset basis
        hash = 0xcbf29ce484222325ULL; 
        // 64-bit FNV-1a prime
        const uint64_t fnv_prime = 0x100000001b3ULL; 
        uint64_t& all_reachable_bitset = stats.all_reachable_bitset; // union of outputs reachable by any switch configuration
        uint8_t&  max_reachable_n = stats.max_reachable_n;
        uint16_t& max_reachable_bitmask = stats.max_reachable_bitmask;
        uint64_t& max_reachable_bitset = stats.max_reachable_bitset;
        for(const auto&[r,m] : reachable_switch_array) {
            all_reachable_bitset |= r;
            hash ^= r;
            hash *= fnv_prime;
            uint8_t cnt = static_cast<uint8_t>((std::bitset<64>(r)).count());
            if (cnt > max_reachable_n) {
                max_reachable_n = cnt;
                max_reachable_bitmask = m;
                max_reachable_bitset = r;
            }
        }
        hash ^= (hash >> 33);
        hash *= 0xff51afd7ed558ccdULL;
        hash ^= (hash >> 33);
        stats.n_never_reached = V - (std::bitset<64>(all_reachable_bitset)).count(); 
        stats.n_unique_configs = static_cast<uint16_t>(reachable_switch_array.size());
        stats.total_generate_counter = total_generate_counter;
        total_generate_counter = 0; //reset counter

        if (verbose) {
            print_stats(stats);

            for(auto&[r, b] : reachable_switch_array) {
                std::cout << "bitmask|reachable: " << mask2string(b, N) << "|" << mask2string(r, V) << std::endl;
            }

            std::cout << "Config: " << get_readable_config() << std::endl;
        }
        return stats;
    }

    int best_choice_of_m_vertices() {
        const auto& ori_arr = reachable_switch_array;
        int max_unique_rows = 0;
        std::vector<uint64_t> best_removal_masks {};
        for(auto[reachable, bitmask] : ori_arr) {
            const int nbit1 = std::popcount(reachable);
            if (nbit1 < M) continue;

            // std::cout << "mask: " << mask2string(bitmask, N) << std::endl;

            // bit 1 positions
            std::vector<int> positions;
            while (reachable > 0) {
                // Find the index of the lowest set bit (0-indexed)
                int pos = std::countr_zero(reachable);
                positions.push_back(pos);
                
                // Clear the lowest set bit using Brian Kernighan's trick
                reachable &= (reachable - 1); 
            }
            assert(nbit1 == (int) positions.size());

            // std::cout << "Shrink the size of reachable from V to nbit1:" << std::endl;
            std::vector<uint64_t> new_arr{}; 
            for(const auto&[reachable, bitmask] : ori_arr) {
                uint64_t new_reachable = 0;
                int bit = 0;
                for (int v : positions) {
                    new_reachable |= ((reachable >> v) & 1ULL) << bit;
                    bit++;
                }
                new_arr.emplace_back(new_reachable);
                // std::cout << "bitmask|reachable: " << mask2string(bitmask, N) << "|" << mask2string(reachable, nbit1) << std::endl;
            }
            

            auto [nrow, masks] = findBestColumnRemoval(new_arr, nbit1, nbit1 - M);

            // std::cout << "After remove cols and keep M cols, the max unique row number: " << nrow << std::endl;
            // std::cout << "best masks (to remove):" << std::endl;
            // for(auto m : masks) {
            //     std::cout << mask2string(m, nbit1) << std::endl;
            // }
            // std::cout << std::endl;

            // convert the best_removal_masks to the real best_removal masks before shrinking
            for(uint64_t &mask: masks) {
                auto old = mask;
                old = (~old) & ((nbit1 >= 64) ? ~0ULL : ((1ULL << nbit1) - 1)); // best keep mask within the nbit1 range
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

        if (verbose){
            std::cout << "\nAfter selecting the best M output from the max reachable bitset, unique configurations: " << max_unique_rows << std::endl;
            std::cout << "Best keep mask(s) (" << best_removal_masks.size() << "): " << std::endl;
            for (auto mask : best_removal_masks) {
                std::cout << mask2string(mask, V) << std::endl;
            }
        }

        return max_unique_rows;
    }

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
    static uint8_t encode_pin(PoleType poleType, PinType pinType, uint8_t idx) { return idx < 16 ? (poleType << 6) | (pinType << 4) | idx : SPECIAL;}
    static std::tuple<PoleType, PinType, uint8_t> decode_pin(uint8_t pin) { return std::make_tuple(static_cast<PoleType>(pin >> 6), static_cast<PinType>((pin >> 4) & 0x3), pin & 0xF);}

    // not tested
    void update_pools(const std::string& s) {
        // Parse readable config format: "3Bb | 0Ab | 0Bc | ..."
        // we only have limited knowledge of wall_pool's first several elements, fill the rest with unique position then
        pool.clear();
        wall_pool.clear(); 
        
        // Split by '|' to get vertices
        std::vector<std::vector<std::string>> vertices;
        size_t pos = 0;
        
        while (pos < s.length()) {
            // Skip whitespace
            while (pos < s.length() && (s[pos] == ' ' || s[pos] == '\n' || s[pos] == '\r' || s[pos] == '\t')) ++pos;
            if (pos >= s.length()) break;
            
            // Find next '|' or end
            size_t next_pipe = s.find('|', pos);
            if (next_pipe == std::string::npos) next_pipe = s.length();
            
            // Extract vertex string
            std::string vertex_str = s.substr(pos, next_pipe - pos);
            
            // Parse pins in this vertex
            std::vector<std::string> pins;
            std::stringstream pin_ss(vertex_str);
            std::string pin_str;
            
            while (pin_ss >> pin_str) {
                pins.push_back(pin_str);
            }
            
            if (!pins.empty()) {
                vertices.push_back(pins);
            }
            
            pos = next_pipe + 1;
        }
        
        // Reconstruct pool from parsed pins
        for (const auto& vertex : vertices) {
            for (const auto& pin_str : vertex) {
                uint8_t pin;
                if (pin_str == "Inp") {
                    pin = INPUT;
                } else {
                    // Parse format: {idx}{poleChar}{pinChar}
                    // e.g., "3Bb" -> idx=3, pole=B, pintype=b
                    // Last char is pin type, second-to-last is pole, rest is idx
                    uint8_t idx = static_cast<uint8_t>(std::stoi(pin_str.substr(0, pin_str.length() - 2)));
                    char pole_char = pin_str[pin_str.length() - 2];
                    char pin_char = pin_str[pin_str.length() - 1];
                    
                    PoleType pole = (pole_char == 'A') ? Pole1 : Pole2;
                    PinType pin_type;
                    if (pin_char == 'a') pin_type = Pin1;
                    else if (pin_char == 'b') pin_type = Pin2;
                    else pin_type = Disconnected;
                    
                    pin = encode_pin(pole, pin_type, idx);
                }
                pool.push_back(pin);
            }
        }
        
        // Reconstruct wall_pool from vertex boundaries
        // wall_pool[i] is the position where vertex i+1 starts
        size_t wall_pos = 1; // Start from position 1 (first internal wall)
        for (size_t i = 0; i < vertices.size() - 1; ++i) {
            wall_pos += vertices[i].size();
            wall_pool.push_back(static_cast<uint8_t>(wall_pos));
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

    static void print_stats(Statistics stats) {
        std::cout << "==================================================================Basic Parameters===================================================================" << std::endl;
        std::cout << "#switch N: " << (int)stats.N << std::endl;
        std::cout << "#output M: " << (int)stats.M << std::endl;
        std::cout << "#vertex V: " << (int)stats.V << std::endl;
        std::cout << "pool size: " << (int)stats.pool_size << std::endl;
        std::cout << "Total tries: " << stats.total_generate_counter << std::endl;
        std::cout << "Unique configurations: " << (int)stats.n_unique_configs << " (" << ((1<<stats.N) - (int)stats.n_unique_configs) << " removed)" << std::endl;
        std::cout << "All reachable bitset: " << mask2string(stats.all_reachable_bitset, stats.V) << std::endl;
        std::cout << "Never reached outputs: " << (int)stats.n_never_reached << std::endl;
        std::cout << "Max n reachable: " << (int)stats.max_reachable_n << std::endl;
        std::cout << "Max bitmask|reachable: " << mask2string(stats.max_reachable_bitmask, stats.N) << "|" << mask2string(stats.max_reachable_bitset, stats.V) << std::endl;
        std::cout << "all bitmasks and reachables: " << std::endl;
        std::cout << "hash value:" << stats.hash << std::endl;
        std::cout << "=====================================================================================================================================================" << std::endl;
    }
};
