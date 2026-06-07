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

#define DEBUG 1
// define a macro to run function only in debug mode
#define DEBUG_RUN(func) do { if (DEBUG) { func; } } while(0)

// Data Encoding Utilities
enum ComponentType { POWER = 0, LED = 1, SWITCH = 2 };
// for power and LED, its just plus and minus. For switch, pin0, pin1, pin2
// swtich is single pole double throw, so pin0 is common, and POS/NEG are the two throws.
enum PinType { POS = 0, NEG = 1, COM = 2 }; 

// struct PinInfo {
//     ComponentType componentType : 2;
//     PinType pinType  : 2;
//     uint8_t compIdx : 4; 
// };

// uint8_t is enough, the simulation will for sure use less than 16 switches and 16 LEDs, and we can encode the component type and pin type in the upper bits.
uint8_t encode_pin(ComponentType compType, PinType pinType, uint8_t idx) {
    assert (idx < 16);
    return (compType << 6) | (pinType << 4) | idx;
}

void decode_pin(uint8_t pin, ComponentType &compType, PinType &pinType, uint8_t &idx) {
    compType = static_cast<ComponentType>(pin >> 6);
    pinType = static_cast<PinType>((pin >> 4) & 0x3);
    idx = pin & 0xF;
}

/*
// Enforce these structural sanity constraints. If violated, re-partition. If re-partitioning fails 100 times, restart from step 1 (re-shuffling):
// - Constraint A: Every vertex must contain 2 or more pins (no isolated or floating single pins).
// - Constraint B: A vertex cannot contain BOTH the Power (+) and Power (-) poles (instant hard short circuit).
// - Constraint C: The same LED's + and - pins, or a switch's Pin 0 and Pin 1/2, cannot be inside the same vertex (renders the component natively useless).
// - Constraint D: An LED's + and - pins cannot span directly between the Power (+) vertex and Power (-) vertex. (This would render the LED permanently on or permanently reverse-biased/off, bypassing the switch matrix).
// - Constraint E: A switch's Pin 0 and either Pin 1 or Pin 2 cannot span directly between the Power (+) vertex and Power (-) vertex (flipping this switch would cause a guaranteed short circuit).
// - Constraint F: Vertices with power pins must have a switch pin connected to it, or else the circuit is not really functional as we won't be able to control the connectivity between power rails.
// - Constraint G: Vertices cannot only contain LED pins
*/
class CircuitGenerator {
public:
    using v2p_t = std::array<std::pair<uint8_t, std::array<uint8_t, 0xFF>>, 16>; // vertex index -> (pin count, pin list)
    using sp2v_t = std::array<std::array<uint8_t, 16>, 3>; // switch pin to vertex index
    using lp2v_t = std::array<std::array<uint8_t, 16>, 2>; // LED pin to vertex index

    CircuitGenerator(uint8_t switches, uint8_t leds, std::vector<uint8_t> vertex_sizes) 
        : N(switches), M(leds), V(vertex_sizes.size()), pool_size(static_cast<uint8_t>(2 + 2 * leds + 3 * switches)), rng(std::random_device{}()), 
          pool(0), parents(pool_size), wall_positions(0), switch_p2v{}, led_p2v{}, power_p2v{}, switch_p2pos{}
    {
        assert(V >= 2); // Need at least 2 vertices to separate power rails
        assert(N < 16 && M < 16 && V < 16); // Ensure component counts are within bounds
        assert(pool_size >= static_cast<uint8_t>(V * 2)); // At least need 2 pins per vertex on average to satisfy the minimum pin count constraint
        std::cout << "Initializing Circuit Generator with " << (int)N << " switches, " << (int)M << " LEDs, and " << (int)V << " vertices. Pool size " << (int)pool_size << std::endl;

        // Populate wall positions based on vertex sizes
        int p = 0;
        wall_positions.push_back(p);
        for (int i =0; i < V; ++i) {
            assert(vertex_sizes[i] >= MIN_WALL_DISTANCE); // Constraint A: Ensure minimum distance between walls to avoid isolated pins
            p += vertex_sizes[i];
            wall_positions.push_back(p);
        }
        assert(wall_positions.back() == pool_size);

        // Set parent to the start of each vertex's pin list
        for (int i = 0; i < V; ++i) {
            for (int j = wall_positions[i]; j < wall_positions[i + 1]; ++j) {
                parents[j] = wall_positions[i];
            }
        }

        // Populate pin pool with encoded pin information
        for (int i = 0; i < M; ++i) {
            pool.push_back(encode_pin(LED, POS, i)); // LED +
            pool.push_back(encode_pin(LED, NEG, i)); // LED -
        }
        for (int i = 0; i < N; ++i) {
            pool.push_back(encode_pin(SWITCH, POS, i)); // Switch Pin 0
            pool.push_back(encode_pin(SWITCH, NEG, i)); // Switch Pin 1
            pool.push_back(encode_pin(SWITCH, COM, i)); // Switch Pin 2
        }
        pool.push_back(encode_pin(POWER, POS, 0)); // Power +
        pool.push_back(encode_pin(POWER, NEG, 0)); // Power -
        assert(pool.size() == pool_size);


    }

    std::tuple<const std::vector<uint8_t>*, const sp2v_t*> generate() {
        stats_total_trys = 0;
        stats_total_accepted = 0;

        while (stats_total_trys < 10000) {
            ++stats_total_trys;
            std::shuffle(pool.begin(), pool.end(), rng);

            // One-pass fill in the vertex index for each switch and LED pin
            for (int i = 0; i < V; ++i) {
                for (int j = wall_positions[i]; j < wall_positions[i + 1]; ++j) {
                    uint8_t pin = pool[j];
                    if (pin >> 6 == SWITCH) {
                        switch_p2v[(pin >> 4) & 0x3][pin & 0xF] = i;
                    } else if (pin >> 6 == LED) {
                        led_p2v[(pin >> 4) & 0x3][pin & 0xF] = i;
                    } else {
                        power_p2v[(pin >> 4) & 0x1] = i; // power pin, either + or -
                    }
                }
            }
            // std::cout << "Total tries: " << stats_total_trys << ", Total accepted: " << stats_total_accepted << std::endl;
            // std::cout << "\nSwitch Pin to Vertex Mapping:";
            // std::cout << "\nSwitch "; for (int i = 0; i < N; ++i) printf("%2d ", i); 
            // std::cout << "\nPinPos "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2v[POS][i]); 
            // std::cout << "\nPinNeg "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2v[NEG][i]); 
            // std::cout << "\nPinCom "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2v[COM][i]); 

            // // print p2v
            // std::cout << "\nLED Pin to Vertex Mapping:";
            // std::cout << "\nLED    "; for (int i = 0; i < M; ++i) printf("%2d ", i); 
            // std::cout << "\nPinPos "; for (int i = 0; i < M; ++i) printf("%2d ", (int)led_p2v[POS][i]); 
            // std::cout << "\nPinNeg "; for (int i = 0; i < M; ++i) printf("%2d ", (int)led_p2v[NEG][i]); 
            // std::cout << std::endl << std::endl;

            bool constraint_violated = false;
            // Constraint B: A vertex cannot contain BOTH the Power (+) and Power (-) poles (instant hard short circuit).
            if (power_p2v[POS] == power_p2v[NEG]) {
                constraint_violated = true;
            }

            // Constraint C, D, E: Evaluate switch and LED pin placements for invalid configurations
            for (int i = 0; i < N && !constraint_violated; ++i) { // constraint C and E evaluation for switches
                if (switch_p2v[POS][i] == switch_p2v[NEG][i] || 
                    switch_p2v[COM][i] == switch_p2v[POS][i] || 
                    switch_p2v[COM][i] == switch_p2v[NEG][i]) {
                    constraint_violated = true;
                    break;
                }
                if ((switch_p2v[COM][i] == power_p2v[POS] && switch_p2v[NEG][i] == power_p2v[NEG]) || 
                    (switch_p2v[COM][i] == power_p2v[POS] && switch_p2v[POS][i] == power_p2v[NEG]) || 
                    (switch_p2v[COM][i] == power_p2v[NEG] && switch_p2v[NEG][i] == power_p2v[POS]) || 
                    (switch_p2v[COM][i] == power_p2v[NEG] && switch_p2v[POS][i] == power_p2v[POS])) {
                    constraint_violated = true;
                    break;
                }
            }
            for (int i = 0; i < M && !constraint_violated; ++i) { // constraint C and D evaluation for LEDs
                if (led_p2v[POS][i] == led_p2v[NEG][i]) {
                    constraint_violated = true;
                    break;
                }
                if ((led_p2v[POS][i] == power_p2v[POS] && led_p2v[NEG][i] == power_p2v[NEG]) || 
                    (led_p2v[NEG][i] == power_p2v[NEG] && led_p2v[POS][i] == power_p2v[POS])) {
                    constraint_violated = true;
                    break;
                }
            }

            // Constraint F: Vertices with power pins must have a switch pin connected to it, or else the circuit is not really functional as we won't be able to control the connectivity between power rails.
            bool power_pos_has_switch = false;
            bool power_neg_has_switch = false;
            for (int i = 0; i < N && !constraint_violated; ++i) {
                if (!power_pos_has_switch && (switch_p2v[POS][i] == power_p2v[POS] || switch_p2v[NEG][i] == power_p2v[POS] || switch_p2v[COM][i] == power_p2v[POS])) {
                    power_pos_has_switch = true;
                }
                if (!power_neg_has_switch && (switch_p2v[POS][i] == power_p2v[NEG] || switch_p2v[NEG][i] == power_p2v[NEG] || switch_p2v[COM][i] == power_p2v[NEG])) {
                    power_neg_has_switch = true;
                }
            }
            if (!power_pos_has_switch || !power_neg_has_switch) {
                constraint_violated = true;
            }

            // Constraint G: Vertices cannot only contain LED pins
            for (int i = 0; i < V && !constraint_violated; ++i) {
                bool has_non_led_pin = false;
                for (int j = wall_positions[i]; j < wall_positions[i + 1]; ++j) {
                    uint8_t pin = pool[j];
                    if (pin >> 6 == SWITCH || pin >> 6 == POWER) {
                        has_non_led_pin = true;
                    }
                    if (has_non_led_pin) break;
                }
                if (!has_non_led_pin) {
                    constraint_violated = true;
                    break;
                }
            }

            if (constraint_violated) {
                continue;
            }
            ++stats_total_accepted;
            // good, we generate a valid circuit!
            
            // Sort each vertex, set DSU's parent to the start of each vertex's pin list, and fill in the switch_p2pos for quick complementary pin lookup during simulation
            for (int i = 0; i < V; ++i) {
                std::sort(pool.begin() + wall_positions[i], pool.begin() + wall_positions[i + 1]); // sort the pins within each vertex for easier constraint evaluation
                for (int j = wall_positions[i]; j < wall_positions[i + 1]; ++j) {
                    parents[j] = wall_positions[i];
                    if (pool[j] >> 6 == SWITCH) {
                        switch_p2pos[(pool[j] >> 4) & 0x3][pool[j] & 0xF] = j;
                    }
                }
            }
            return {&pool, &switch_p2pos};
        }
        std::cerr << "Failed to generate a valid circuit after " << stats_total_trys << " tries " << std::endl;
        return {nullptr, nullptr};
    }

    void print_current_configuration() {
        std::cout << "Current Circuit Configuration" << std::endl;
        std::cout << "Total tries: " << stats_total_trys << ", Total accepted: " << stats_total_accepted << std::endl;
        // print p2v
        std::cout << "\nSwitch Pin to Vertex Mapping:";
        std::cout << "\nSwitch "; for (int i = 0; i < N; ++i) printf("%2d ", i); 
        std::cout << "\nPinPos "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2v[POS][i]); 
        std::cout << "\nPinNeg "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2v[NEG][i]); 
        std::cout << "\nPinCom "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2v[COM][i]); 
        std::cout << std::endl;

        // print p2v
        std::cout << "\nLED Pin to Vertex Mapping:";
        std::cout << "\nLED    "; for (int i = 0; i < M; ++i) printf("%2d ", i); 
        std::cout << "\nPinPos "; for (int i = 0; i < M; ++i) printf("%2d ", (int)led_p2v[POS][i]); 
        std::cout << "\nPinNeg "; for (int i = 0; i < M; ++i) printf("%2d ", (int)led_p2v[NEG][i]); 
        std::cout << std::endl;

        // print p2pos for switches
        std::cout << "\nSwitch Pin to Position in Pool:";
        std::cout << "\nSwitch "; for (int i = 0; i < N; ++i) printf("%2d ", i); 
        std::cout << "\nPinPos "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2pos[POS][i]); 
        std::cout << "\nPinNeg "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2pos[NEG][i]); 
        std::cout << "\nPinCom "; for (int i = 0; i < N; ++i) printf("%2d ", (int)switch_p2pos[COM][i]);
        std::cout << std::endl;

        std::cout << get_v2p_string(get_v2p()) << std::endl;
    }

    const v2p_t& get_v2p() {
        for (int i = 0; i < V; ++i) {
            v2p[i].first = 0;
            for (int j = wall_positions[i]; j < wall_positions[i + 1]; ++j) {
                v2p[i].second[v2p[i].first++] = pool[j];
            }
        }
        return v2p;
    }

    const std::vector<uint8_t>& get_parents() const {
        return parents;
    }

    static std::string get_v2p_string(const v2p_t& v2p) {
        std::string result = "v2p:";
        for (int i = 0; i < v2p.size(); ++i) {
            if (v2p[i].first == 0) continue; // skip empty vertices
            result += "[" + std::to_string(i) + "]";
            for (int j = 0; j < v2p[i].first; ++j) {
                uint8_t pin = v2p[i].second[j];
                ComponentType compType;
                PinType pinType;
                uint8_t idx;
                decode_pin(pin, compType, pinType, idx);
                result += "(" + std::string(1, (compType == POWER) ? 'P' : (compType == LED) ? 'L' : 'S') +
                          std::string(1, (pinType == POS) ? '+' : (pinType == NEG) ? '-' : '@') +
                          std::to_string(idx) + ")";
            }
        }
        return result;
    }

    static std::string get_vp2_oneline_string(const v2p_t& v2p) {
        std::string result = "v2p:";
        for (int i = 0; i < v2p.size(); ++i) {
            if (v2p[i].first == 0) continue; // skip empty vertices
            result += "[" + std::to_string(i) + "]";
            for (int j = 0; j < v2p[i].first; ++j) {
                uint8_t pin = v2p[i].second[j];
                ComponentType compType;
                PinType pinType;
                uint8_t idx;
                decode_pin(pin, compType, pinType, idx);
                result += "(" + std::string(1, (compType == POWER) ? 'P' : (compType == LED) ? 'L' : 'S') +
                          std::string(1, (pinType == POS) ? '+' : (pinType == NEG) ? '-' : '@') +
                          std::to_string(idx) + ")";
            }
            result += ",";
        }
        return result;
    }   

private:
    static constexpr uint8_t MIN_WALL_DISTANCE = 2; // Minimum distance between walls to ensure no vertex is completely isolated
    uint8_t N, M, V, pool_size;
    std::mt19937 rng; 
    std::vector<uint8_t> pool;
    std::vector<uint8_t> parents;
    std::vector<uint8_t> wall_positions;

    // Map from pin to vertex index for quick constraint evaluation
    sp2v_t switch_p2v; // the switch's pin belong to which vertex, used to evaluate constraint C and E
    lp2v_t led_p2v; // the LED's pin belong to which vertex, used to evaluate constraint C and D
    std::array<uint8_t, 2> power_p2v; // the power pins belong to which vertex, used to evaluate constraint B
    sp2v_t switch_p2pos; // the switch's pin position in the pool, used to quickly find the complementary pin position for constraint evaluation
    v2p_t v2p; // auxiliary structure to store vertex to pin mapping for quick lookup during simulation

    // counters for print_current_configuration()
    int stats_total_trys = 0;
    int stats_total_accepted = 0;

};
