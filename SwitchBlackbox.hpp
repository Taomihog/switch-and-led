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
#include <memory>
#include <algorithm>
#include <ranges>
#include <format>

#include "DSU_uint8.hpp"
#include "utils.hpp"
#include "TruthTable.hpp"
#include "Matrix.hpp"
#include "utils.hpp"

#define DEBUG 0
// define a macro to run function only in debug mode
#define DEBUG_RUN(func) do { if (DEBUG) { func; } } while(0)


/*
Consider this simulation:
We have a power INPUT go into a blackbox, the blockbox has N switches and M output. The switches are double pole double throw.
We generate vertices of pins, the vertices size is an INPUT, 
*/
class SwitchBlackbox {
public:
    std::array<size_t,20> fc; // count failed times at each filter

    SwitchBlackbox(uint8_t n_switches, uint8_t n_outputs, uint8_t n_vertices):
        N(n_switches), M(n_outputs), V(n_vertices), rng(std::random_device{}()), pool_size(static_cast<uint8_t>(1 + 6 * n_switches)), 
        pool(), wall_pool(), fc{}
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

    void generate_by_string(std::string s) {
        size_t hash2_from_file = load_pools_from_string(s);
        assert(generate() && "Error: generate() failed: the read is not a valid circuit.");
        assert(tt_hash() == hash2_from_file && "tt_hash mismatch. Data loading error. Check data integrity.");
    }

    void generate_random() {
        do{
        std::shuffle(pool.begin(), pool.end(), rng);
        std::shuffle(wall_pool.begin() , wall_pool.end(), rng);
        std::sort(wall_pool.begin(), wall_pool.begin() + V - 1); // Sort wall_pool[0] through wall_pool[V-2] (the V-1 internal walls we use)
        } while (!generate());
    }

    void generate_dry() {
        generate();
    }

    bool filter() {
        std::vector<uint64_t> m (tt->outputs.begin(), tt->outputs.end());

        // Filter 1: unqiue rows == 1 << N
        std::ranges::sort(m);
        if (std::unique(m.begin(), m.end()) - m.begin() < (1 << N) && ++fc[1]) return false;

        // Filter 2: n_rel >= M
        std::tie(mask_relevant, this->relevance) = TruthTable::relevant_bits(*tt);
        int n_rel = std::popcount(mask_relevant);
        if (n_rel < M && ++fc[2]) return false;

        // Filter 3: packed matrix using mask_relevant have 1 << N unique rows
        m = Matrix::pack_matrix(m, mask_relevant);
        if (std::unique(m.begin(), m.end()) - m.begin() < (1 << N) && ++fc[3]) return false;

        //Filter 4: best Retention can find solution
        best_masks.clear();
        for(auto r : m) {
            if (std::popcount(r) < M) continue; // Here I actually enforce a row to have all bit 1s. 
            auto [n_max_unqiue_rows, masks] = Matrix::findBestColumnRetention(m, r, M);
            if (n_max_unqiue_rows == 1 << N) best_masks.insert(best_masks.end(), masks.begin(), masks.end());
        }
        if (best_masks.size() == 0 && ++fc[4]) return false;
        
        std::ranges::for_each(best_masks, [this](uint64_t& packed) { packed = Matrix::unpack_mask(packed, mask_relevant);});

        return true;
    }

    struct Statistics {
        struct tt_properties {
            uint64_t mask;
            size_t sumrel;
            size_t sum1;
            size_t sum2;
            size_t sd;
        };

        size_t hash1;
        size_t hash2;
        std::string human_readable;
        std::vector<tt_properties> ttp;
    };

    Statistics stats(bool verbose = false) const{
        Statistics stats;
        stats.hash1 = pool_hash(); 
        stats.hash2 = tt_hash(); 
        stats.human_readable = get_readable_config();

        // check for each best_mask and find good parameters. If a target mask is provided, only calculate that target mask's properties
        for (auto mask : best_masks) {

            std::vector<uint64_t> mat (tt->outputs.begin(), tt->outputs.end());
            mat = Matrix::pack_matrix(mat, mask);

            std::vector<std::vector<size_t>> rel;
            std::vector<size_t> total(V, 0);
            for(size_t i = 0; i < V; ++i) {
                if (mask >> i & 1) {
                    rel.emplace_back(relevance[i].begin(), relevance[i].end());
                    const auto& row = rel.back();
                    bool has_zero = std::ranges::any_of(row, [](int n) { return n == 0; });
                    if(has_zero) {
                        std::cerr << "Bug: inconsistent relevance value with mask" << std::endl;
                        std::cerr << "mask_relevant: " << utils::mask2string(mask_relevant, V) << std::endl;
                        std::cerr << "current mask: " << utils::mask2string(mask, V) << std::endl;
                        exit(0);
                    }
                    total[i] = std::accumulate(row.begin(), row.end(), 0);
                }
            }
            const size_t sumrel = std::accumulate(total.begin(), total.end(), 0);
            const tt64_t tt_temp = tt64_t(N, M, mat); // total states 1<<N and use the rightmost M bits in output
            const size_t sum1 = TruthTable::sum_output_distance_for_adjacent_inputs(1, tt_temp);
            const size_t sum2 = TruthTable::sum_output_distance_for_adjacent_inputs(2, tt_temp);
            const size_t sd = Matrix::stdev(mat);
            stats.ttp.emplace_back(mask, sumrel, sum1, sum2, sd);
        }

        return stats;
    }

    void print_human_readable() {
        // print all pins in pool
        auto pin2string = [](uint8_t pin) -> std::string {
            if (pin == INPUT) return "Inp";
            auto [poleType, pinType, idx] = decode_pin(pin);
            char poleChar = (poleType == Pole1) ? 'A' : 'B';
            char pinChar = (pinType == Pin1) ? '+' : (pinType == Pin2) ? '-' : 'o';
            return std::to_string(idx) + poleChar + pinChar;
        };
        std::cout << "pins in pool:      ";
        for (size_t i = 0; i < pool.size(); ++i) {
            uint8_t pin = pool[i];
            std::cout << pin2string(pin) << " ";
        }
        std::cout << std::endl;
        std::cout << "pins in vertices: ";
        for (size_t i = 0; i < pool.size(); ++i) {
            printf(" %2d ", pos2vertex[i]);
        }
        std::cout << std::endl;
    }

    void print_pretty_final_format(std::string config_str, uint64_t mask) {
        // pipeline to create the final result (and print) from a given config string and a mask
        // a good format for double-check the result and guidance to connect switches
        // print truth table:
        //      left size all values <-> right side masked
        std::cout << "config string:" << config_str << std::endl;
        generate_by_string(config_str);
        assert(filter());

        std::vector<uint64_t> mat (tt->outputs.begin(), tt->outputs.end());
        mat = Matrix::pack_matrix(mat, mask);

        std::vector<std::vector<size_t>> rel;
        std::vector<size_t> total(V, 0);
        for(size_t i = 0; i < V; ++i) {
            if (mask >> i & 1) {
                rel.emplace_back(relevance[i].begin(), relevance[i].end());
                const auto& row = rel.back();
                bool has_zero = std::ranges::any_of(row, [](int n) { return n == 0; });
                if(has_zero) {
                    std::cerr << "Bug: inconsistent relevance value with mask" << std::endl;
                    std::cerr << "mask_relevant: " << utils::mask2string(mask_relevant, V) << std::endl;
                    std::cerr << "current mask: " << utils::mask2string(mask, V) << std::endl;
                    exit(0);
                }
                total[i] = std::accumulate(row.begin(), row.end(), 0);
            }
        }
        const size_t sumrel = std::accumulate(total.begin(), total.end(), 0);
        const tt64_t tt_temp = tt64_t(N, M, mat); // total states 1<<N and use the rightmost M bits in output
        const size_t sum1 = TruthTable::sum_output_distance_for_adjacent_inputs(1, tt_temp);
        const size_t sum2 = TruthTable::sum_output_distance_for_adjacent_inputs(2, tt_temp);
        const size_t sd = Matrix::stdev(mat);

        std::cout << "==============Details==============" << std::endl;
        std::cout << "#switch N:             " << (int)N << std::endl;
        std::cout << "#output M:             " << (int)M << std::endl;
        std::cout << "#vertex V:             " << (int)V << std::endl;
        std::cout << "pool size:             " << (int)pool_size << std::endl;
        std::cout << "number of total masks: " << best_masks.size() << std::endl;
        std::cout << "current mask:" << utils::mask2string(mask, V) << std::endl;
        std::cout << "after masking:" << std::endl;
        std::cout << "  sum of rel table: " << sumrel << std::endl;
        std::cout << "  sum dist 1:       " << sum1 << std::endl;
        std::cout << "  sum dist 2:       " << sum2 << std::endl;
        std::cout << "  stdev:            " << sd << std::endl;
        std::cout << "===================================" << std::endl;
        std::cout << "truth table before masking:" << std::endl;
        for(size_t x = 0; x < 1 << N; ++x) {
            std::cout << "input|output: " << utils::mask2string(x, N) << "|" << utils::mask2string(tt->outputs[x], V) << std::endl;
        }
        std::cout << "===================================" << std::endl;
        std::cout << "truth table after masking:" << std::endl;
        for(size_t x = 0; x < 1 << N; ++x) {
            std::cout << "input|output: " << utils::mask2string(x, N) << "|" << utils::mask2string(mat[x], M) << std::endl;
        }
        std::cout << "===================================" << std::endl;
        std::cout << std::endl << "relevance table before masking:" << std::endl;
        for(size_t i = 0; i < relevance.size(); ++i) {
            for (auto v : relevance[i]) { printf("%3d ", v); }
            std::cout << ((mask_relevant >> i & 1) ? "  V" : "  X") << std::endl;
        }
        std::cout << "===================================" << std::endl;
        std::cout << "relevant matrix after masking:" << std::endl;
        for (const auto& row: rel) {
            for(auto v : row) { printf("%3d ", v); }
            std::cout << std::endl;
        }
        std::cout << "===================================" << std::endl;
        print_human_readable();
        std::cout << "kept vertices: ";
        // Pay extra attention if this is correct. The i-th bit should be the i-th vertex
        for(int i = 0; i < V; ++i) {
            if (mask >> i & 1) std::cout << i << ", ";
        }
        std::cout << std::endl;
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

    using tt64_t = TruthTable::tt_t<uint64_t>;
    using p2v_t = std::array<uint8_t, 0xFF>; // switch pin to vertex index

    // 3. data from generate(), they are constant outside of generate() but need to be stored as member variables for easy access in both constraint validation and simulation
    std::unique_ptr<const tt64_t> tt; // map N input switch to V output vertices
    std::vector<uint8_t> pos2vertex; // pool position -> vertex index mapping

    // 4. filter generate 2 important vectors
    std::vector<std::vector<size_t>> relevance;
    std::vector<uint64_t> best_masks;
    uint64_t mask_relevant = ~0ULL;
    
    enum PoleType { Pole1 = 0, Pole2 = 1 };
    enum PinType { Pin1 = 0, Pin2 = 1, Disconnected = 2 }; 
    static constexpr uint8_t INPUT = 2 << 6 | 3 << 4; 
    static constexpr uint8_t SPECIAL = 3 << 6 | 3 << 4; // special won't belong to pool or wall_pool (pool_size will be less than 16*3+1, so wall_pool size < 16*3)
    // uint8_t is enough, the simulation will for sure use less than 16 switches and 16 LEDs, and we can encode the component type and pin type in the upper bits.
    static uint8_t encode_pin(PoleType poleType, PinType pinType, uint8_t idx) { return idx < 16 ? (poleType << 6) | (pinType << 4) | idx : SPECIAL;}
    static std::tuple<PoleType, PinType, uint8_t> decode_pin(uint8_t pin) { return std::make_tuple(static_cast<PoleType>(pin >> 6), static_cast<PinType>((pin >> 4) & 0x3), pin & 0xF);}

    // core function to generate the circuit and TruthTable
    bool generate() {
        // step 1. generate circuit
        std::vector<uint8_t> vertex_sizes(V); 
        std::vector<uint8_t> parents(pool_size); // DSU parent array for connectivity simulation
        p2v_t p2pos; // pins to positions in pool
        p2v_t p2v; // pins to vertices
        uint8_t input_pos; // position of the power INPUT pin in the pool
        uint8_t input_vertex; // vertex index of the power INPUT pin, used to quickly check if an output is powered in the simulation
        // For checking loaded data integrity, this optional store tt_hash from loaded data
        ++fc[0];

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
        if(!validate_constraints(N, vertex_sizes, p2v, input_vertex)) return false;

        // step 2. generate_tt
        // the circuit is determined by pool and wall positions (or parents, or p2pos),
        // now we want to calculate what vertices will be reachable at different switch configs
        // std::cout << "[debug]" << std::endl;
        std::vector<uint64_t> temp;
        for(uint16_t bitmask = 0; bitmask < (1 << N); ++bitmask) {
            DSU_uint8 dsu(std::vector<uint8_t>{parents});
            // std::cout << "bitmask: " << utils::mask2string(bitmask, N) << std::endl;
            // for (auto v : dsu.parent) {
            //     std::cout << " " << (int)v;
            // }
            // std::cout << std::endl;
            for(int i = 0; i < N; ++i) {
                dsu.unite(p2pos[encode_pin(Pole1, Disconnected, i)], ((bitmask >> i) & 1) == 0 ? p2pos[encode_pin(Pole1, Pin1, i)] : p2pos[encode_pin(Pole1, Pin2, i)]);
                dsu.unite(p2pos[encode_pin(Pole2, Disconnected, i)], ((bitmask >> i) & 1) == 0 ? p2pos[encode_pin(Pole2, Pin1, i)] : p2pos[encode_pin(Pole2, Pin2, i)]);
            }
            // for (auto v : dsu.parent) {
            //     std::cout << " " << (int)v;
            // }
            // std::cout << std::endl;
            // for (int i = 0; i < dsu.parent.size(); ++i) {
            //     std::cout << " " << (int)dsu.find(i);
            // }
            // std::cout << std::endl;

            // each vertex can be an output
            // the original vertices united with the INPUT vertex are set to 1, other vertices are 0
            const uint8_t input_root = dsu.find(input_pos);
            // std::cout << "input_pos/input_root" << (int)input_pos << "," << (int)input_root << std::endl;
            uint64_t reachable_bitset = 0ULL;
            for (size_t i = 0; i < pool.size(); ++i) {
                if (dsu.find(i) == input_root) {
                    auto pin = pool[i];
                    if (pin == INPUT) continue;
                    uint8_t v = p2v[pin];
                    // std::cout << (int)v << ' ';
                    assert(v < V); // guard against invalid vertex index
                    reachable_bitset |= 1ULL << v;// Important!!! reachable_bitset is "reversed", the first bit to the right is vertex 0, the 2nd bit to the right is v 1, ..
                    // It doesn't affect the truthtable, but in print_pretty_final_format(), it does affect how to convert the mask back to vertex
                }
            }
            // std::cout << std::endl;
            // std::cout << utils::mask2string(reachable_bitset, V) << std::endl;
            temp.emplace_back(reachable_bitset);
        }
        tt.reset(new tt64_t {N, V, temp});
        // exit(0);
        return true;
    }

    // physical constraints
    static bool validate_constraints(const int N, const std::vector<uint8_t>& vertex_sizes, const p2v_t& p2v, const uint8_t input_vertex) {
        // power vertex size must > 1
        if (vertex_sizes[input_vertex] <= 1) {
            return false;
        }
        // std::cout << "[debug] " << (int)p2v[encode_pin(Pole2, Pin1, 1)] << ", " << (int)p2v[encode_pin(Pole2, Disconnected, 1)] << std::endl;
        // for each pole of each switch, 2 pins cannot be in the same vertex
        for (int i = 0; i < N; ++i) {
            if (p2v[encode_pin(Pole1, Pin1, i)]         == p2v[encode_pin(Pole1, Pin2, i)] || 
                p2v[encode_pin(Pole1, Disconnected, i)] == p2v[encode_pin(Pole1, Pin1, i)] || 
                p2v[encode_pin(Pole1, Disconnected, i)] == p2v[encode_pin(Pole1, Pin2, i)] ||
                p2v[encode_pin(Pole2, Pin1, i)]         == p2v[encode_pin(Pole2, Pin2, i)] ||
                p2v[encode_pin(Pole2, Disconnected, i)] == p2v[encode_pin(Pole2, Pin1, i)] ||
                p2v[encode_pin(Pole2, Disconnected, i)] == p2v[encode_pin(Pole2, Pin2, i)]) {
                    // std::cout << "pin " << (int) i << "is true" << std::endl;
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
    
    static uint8_t word2uint8(std::string pin_str) {
        assert(pin_str.length() == 3);
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
            if (pin_char == '+') pin_type = Pin1;
            else if (pin_char == '-') pin_type = Pin2;
            else pin_type = Disconnected;
            
            pin = encode_pin(pole, pin_type, idx);
        }
        return pin;
    }
    
    // text: (5, 8, 15) 4Ab 0Aa Inp 4Bc 0Ba 1Bb 2Ac 1Ac 4Ba | Walls: 1, 2, 6, 9 | Hash1: ull | Hash2: ull
    std::string get_readable_config() const { // for data saving
        assert(tt && "tt is not initialized! Shouldn't call get_human_readable.");
        std::stringstream ss;
        ss << '(' << std::to_string(N) << ", " << std::to_string(M) << ", " << std::to_string(V) << ") ";
        // print all pins in pool
        auto pin2string = [](uint8_t pin) -> std::string {
            if (pin == INPUT) return "Inp";
            auto [poleType, pinType, idx] = decode_pin(pin);
            char poleChar = (poleType == Pole1) ? 'A' : 'B';
            char pinChar = (pinType == Pin1) ? '+' : (pinType == Pin2) ? '-' : 'o';
            return std::to_string(idx) + poleChar + pinChar;
        };
            
        for (size_t i = 0; i < pool.size(); ++i) {
            ss << pin2string(pool[i]) << " ";
        }
        ss << "| Walls:";
        for (size_t i = 0; i < V - 1; ++i) {
            ss << " " << std::to_string(static_cast<int>(wall_pool[i]));
        }
        ss << " | Hash1: " << pool_hash() << " | Hash2: " << tt_hash(); 
        return ss.str();
    }
    
    // text: (5, 8, 15) 4Ab 0Aa Inp 4Bc 0Ba 1Bb 2Ac 1Ac 4Ba | Walls: 1, 2, 6, 9 | Hash1: ull | Hash2: ull
    size_t load_pools_from_string(const std::string& input) {
        // text: (5, 8, 15) 4Ab 0Aa Inp 4Bc 0Ba 1Bb 2Ac 1Ac 4Ba | Walls: 1, 2, 6, 9 | Hash1: XXXX | Hash2: XXXX
        pool.clear();
        wall_pool.clear();

        // 1. Isolate the Walls metadata from the main data stream
        size_t wall_marker = input.find("| Walls:");
        assert(wall_marker != std::string::npos);
        const std::string data_part = input.substr(0, wall_marker);
        size_t hash1_marker = input.find("| Hash1:");
        assert(hash1_marker != std::string::npos);
        const std::string wall_part = input.substr(wall_marker + 8, hash1_marker - wall_marker - 8);
        // std::cout << "wall part: " << wall_part << std::endl;
        size_t hash2_marker = input .find("| Hash2:");
        assert(hash2_marker != std::string::npos);
        // std::cout << "hash1 string: " << input.substr(hash1_marker + 8, hash2_marker - hash1_marker - 8) << std::endl;
        // std::cout << "hash2 string: " << input.substr(hash2_marker + 8) << std::endl;
        size_t hash1 = std::stoull(input.substr(hash1_marker + 8));
        size_t hash2 = std::stoull(input.substr(hash2_marker + 8));
        
        // 2. Parse the (v1, v2, v3) tuple from the beginning
        std::stringstream data_ss(data_part);
        char discard; // To skip '(', ')', and ','
        int v1, v2, v3;
        
        if (data_ss >> discard >> v1 >> discard >> v2 >> discard >> v3 >> discard) {
            assert (N == static_cast<uint8_t>(v1) || M == static_cast<uint8_t>(v2) || V == static_cast<uint8_t>(v3));
        }

        // 3. Parse all the words sequentially (No pipe checks needed!)
        std::string word;
        while (data_ss >> word) {
            pool.push_back(word2uint8(word));
        }

        // 4. Parse the wall indices from the trailing portion
        std::stringstream wall_ss(wall_part);
        int wall_idx;
        
        while (wall_ss >> wall_idx) {
            wall_pool.push_back(wall_idx);
            // wall_ss >> discard; // Absorb the trailing comma
        }
        assert(wall_pool.size() == V - 1);

        for (uint8_t i = 1; i < pool_size; ++i) {
            bool already_used = false;
            for (uint8_t w : wall_pool) {
                if (w == i) {
                    already_used = true;
                    break;
                }
            }
            if (!already_used) {
                wall_pool.push_back(i);
            }
        }
        assert(wall_pool.size() == pool_size - 1);

        assert(hash1 == pool_hash() && "Hash1 mismatch, check data integrity.");
        // std::cout << "hash match!" << std::endl;
        return hash2;
    }
    
    // May or may not be used but better to have
    size_t pool_hash() const {
        // 1. Calculate total buffer size
        // 4 bytes for constants + 8 bytes for size of pool + pool data + 8 bytes for size of wall_pool + wall_pool data
        size_t total_size = 4 
                            + pool.size() 
                            + V - 1;
        
        std::vector<uint8_t> buffer;
        buffer.reserve(total_size);

        // 2. Serialize Fixed Constants
        buffer.push_back(N);
        buffer.push_back(M);
        buffer.push_back(V);
        buffer.push_back(pool_size);

        // 3. Serialize 'pool' (6N + 1 elements)
        size_t pool_len = pool.size();
        buffer.insert(buffer.end(), pool.begin(), pool.end());

        // 4. Serialize 'wall_pool' (V - 1 elements)
        size_t wall_pool_len = wall_pool.size();
        buffer.insert(buffer.end(), wall_pool.begin(), wall_pool.begin() + V - 1);
        assert(buffer.size() == 4 + 6 * N + 1 + V - 1);
        // 5. Calculate Hash (Using standard std::hash over the string_view of the buffer)
        std::string_view buffer_view(reinterpret_cast<const char*>(buffer.data()), buffer.size());
        size_t hash_value = std::hash<std::string_view>{}(buffer_view);

        return hash_value;
    }

    size_t tt_hash() const {
        std::string_view buffer_view(reinterpret_cast<const char*>(tt->outputs.data()), tt->outputs.size() * sizeof(uint64_t) / sizeof(char));
        size_t hash_value = std::hash<std::string_view>{}(buffer_view);
        return hash_value;
    }

};
