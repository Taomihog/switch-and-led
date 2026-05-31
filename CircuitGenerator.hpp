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

// N, M, V record the number of Switches, LEDs, and Vertices respectively.
// elements is the shuffled list of all pins. the power (+) and power (-) are on the 2 ends
// complementElementPos records the positions of the complementary pins
//     For LED, pin POS is complementary to pin NEG
//     For switch, pin 0 is complementary to pin POS and pin NEG
// dsu is the disjoint set union structure to quickly evaluate connectivity of vertices.
struct Circuit {
    uint8_t N, M, V;
    const std::vector<uint8_t> pins; // size of pins = 2 + 2*M + 3*N
    std::vector<uint8_t> complementPinPositions; //uint8_t is enough to record positions since we have at most 256 pins, it is computed at the end
};

/*
// Enforce these structural sanity constraints. If violated, re-partition. If re-partitioning fails 100 times, restart from step 1 (re-shuffling):
// - Constraint A: Every vertex must contain 2 or more pins (no isolated or floating single pins).
// - Constraint B: A vertex cannot contain BOTH the Power (+) and Power (-) poles (instant hard short circuit).
// - Constraint C: The same LED's + and - pins, or a switch's Pin 0 and Pin 1/2, cannot be inside the same vertex (renders the component natively useless).
// - Constraint D: An LED's + and - pins cannot span directly between the Power (+) vertex and Power (-) vertex. (This would render the LED permanently on or permanently reverse-biased/off, bypassing the switch matrix).
// - Constraint E: A switch's Pin 0 and either Pin 1 or Pin 2 cannot span directly between the Power (+) vertex and Power (-) vertex (flipping this switch would cause a guaranteed short circuit).
// - Constraint F: Vertex 0 Must have a switch pin connected to it, and Vertex V-1 must have a switch pin connected to it, or else the circuit is not really functional as we won't be able to control the connectivity between power rails.
*/
class CircuitGenerator {
public:
    using v2p_t = std::array<std::pair<uint8_t, std::array<uint8_t, 0xFF>>, 16>; // vertex index -> (pin count, pin list)
    using sp2v_t = std::array<std::array<uint8_t, 16>, 3>; // switch pin to vertex index
    using lp2v_t = std::array<std::array<uint8_t, 16>, 2>; // LED pin to vertex index

    CircuitGenerator(uint8_t switches, uint8_t leds, uint8_t vertices) 
        : N(switches), M(leds), V(vertices), pool_size(static_cast<uint8_t>(2 + 2 * leds + 3 * switches)), rng(std::random_device{}()), 
          pool(0), wall_pool(0), parents(pool_size), switch_p2v{}, led_p2v{}, switch_p2pos{}
    {
        assert(V >= 2); // Need at least 2 vertices to separate power rails
        assert(N < 16 && M < 16 && V < 16); // Ensure component counts are within bounds
        assert(pool_size >= static_cast<uint8_t>(V * 2)); // At least need 2 pins per vertex on average to satisfy the minimum pin count constraint
        std::cout << "Initializing Circuit Generator with " << (int)N << " switches, " << (int)M << " LEDs, and " << (int)V << " vertices. Pool size " << (int)pool_size << std::endl;
        pool.reserve(pool_size); // Reserve space for all pins
        wall_pool.reserve(pool_size - 3); 

        // Populate all pins
        pool.push_back(encode_pin(POWER, POS, 0)); // Power +
        for (int i = 0; i < M; ++i) {
            pool.push_back(encode_pin(LED, POS, i)); // LED +
            pool.push_back(encode_pin(LED, NEG, i)); // LED -
        }
        for (int i = 0; i < N; ++i) {
            pool.push_back(encode_pin(SWITCH, POS, i)); // Switch Pin 0
            pool.push_back(encode_pin(SWITCH, NEG, i)); // Switch Pin 1
            pool.push_back(encode_pin(SWITCH, COM, i)); // Switch Pin 2
        }
        pool.push_back(encode_pin(POWER, NEG, 0)); // Power -
        assert(pool.size() == pool_size);

        for (uint8_t pos = 1; pos < pool_size - 2; ++pos) {
            wall_pool.push_back(pos);
        }
    }

    std::tuple<const std::vector<uint8_t>*, const std::vector<uint8_t>*, const sp2v_t*> generate() {
        stats_total_trys = 0;
        stats_total_accepted = 0;

        while (stats_total_trys < 10000) {
            ++stats_total_trys;
            std::shuffle(pool.begin() + 1, pool.end() - 1, rng);

            // One-pass fill in the vertex index for each switch and LED pin
            for (int i = 0; i < V; ++i) {
                int start = (i == 0) ? 0 : wall_pool[i - 1] + 1;
                int end = (i == V - 1) ? pool_size : wall_pool[i] + 1;
                for (int j = start; j < end; ++j) {
                    uint8_t pin = pool[j];
                    if (pin >> 6 == SWITCH) {
                        switch_p2v[(pin >> 4) & 0x3][pin & 0xF] = i;
                    } else if (pin >> 6 == LED) {
                        led_p2v[(pin >> 4) & 0x3][pin & 0xF] = i;
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
            for (int i = 0; i < N; ++i) { // constraint C and E evaluation for switches
                if (switch_p2v[POS][i] == switch_p2v[NEG][i] || 
                    switch_p2v[COM][i] == switch_p2v[POS][i] || 
                    switch_p2v[COM][i] == switch_p2v[NEG][i]) {
                    constraint_violated = true;
                    break;
                }
                if ((switch_p2v[COM][i] == 0 && switch_p2v[NEG][i] == V - 1) || 
                    (switch_p2v[COM][i] == 0 && switch_p2v[POS][i] == V - 1) || 
                    (switch_p2v[NEG][i] == 0 && switch_p2v[COM][i] == V - 1) || 
                    (switch_p2v[POS][i] == 0 && switch_p2v[COM][i] == V - 1)) {
                    constraint_violated = true;
                    break;
                }
            }
            for (int i = 0; i < M && !constraint_violated; ++i) { // constraint C and D evaluation for LEDs
                if (led_p2v[POS][i] == led_p2v[NEG][i]) {
                    constraint_violated = true;
                    break;
                }
                if ((led_p2v[POS][i] == 0 && led_p2v[NEG][i] == V - 1) || 
                    (led_p2v[NEG][i] == 0 && led_p2v[POS][i] == V - 1)) {
                    constraint_violated = true;
                    break;
                }
            }
            bool vertex_0_has_switch = false;
            bool vertex_v1_has_switch = false;
            for (int i = 0; i < N && !constraint_violated; ++i) { // constraint F evaluation
                if (switch_p2v[COM][i] == 0) {
                    vertex_0_has_switch = true;
                }
                if (switch_p2v[COM][i] == V - 1) {
                    vertex_v1_has_switch = true;
                }
            }
            if (!vertex_0_has_switch || !vertex_v1_has_switch) {
                constraint_violated = true;
            }
            if (constraint_violated) {
                continue;
            }
            ++stats_total_accepted;
            // good, we generate a valid circuit!
            
            DSU_uint8 dsu(pool_size);
            // Sort each vertex, set DSU's parent to the start of each vertex's pin list, and fill in the switch_p2pos for quick complementary pin lookup during simulation
            for (int i = 0; i < V; ++i) {
                int start = (i == 0) ? 0 : wall_pool[i - 1] + 1;
                int end = (i == V - 1) ? pool_size : wall_pool[i] + 1;
                std::sort(pool.begin() + start, pool.begin() + end); // sort the pins within each vertex for easier constraint evaluation
                for (int j = start; j < end; ++j) {
                    parents[j] = start;
                    if (pool[j] >> 6 == SWITCH) {
                        switch_p2pos[(pool[j] >> 4) & 0x3][pool[j] & 0xF] = j;
                    }
                }
            }
            return {&pool, &parents, &switch_p2pos};
        }
        std::cerr << "Failed to generate a valid circuit after " << stats_total_trys << " tries " << std::endl;
        return {nullptr, nullptr, nullptr};
    }

    v2p_t print_current_configuration() {
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

        // fill v2p and print out
        v2p_t v2p{};
        for (int i = 0; i < V; ++i) {
            int start = (i == 0) ? 0 : wall_pool[i - 1] + 1;
            int end = (i == V - 1) ? pool_size : wall_pool[i] + 1;
            v2p[i].first = 0;
            for (int j = start; j < end; ++j) {
                v2p[i].second[v2p[i].first++] = pool[j];
            }
        }
        print_v2p(v2p);
        return v2p;
    }

    static void print_v2p(const v2p_t& v2p) {
        std::cout << "\nVertex to Pin Mapping:";
        for (int i = 0; i < v2p.size(); ++i) {
            if (v2p[i].first == 0) continue; // skip empty vertices
            std::cout << "\nVertex " << i << ": ";
            for (int j = 0; j < v2p[i].first; ++j) {
                uint8_t pin = v2p[i].second[j];
                ComponentType compType;
                PinType pinType;
                uint8_t idx;
                decode_pin(pin, compType, pinType, idx);
                printf("(%c%c%d) ", (compType == POWER) ? 'P' : (compType == LED) ? 'L' : 'S', 
                                    (pinType == POS) ? '+' : (pinType == NEG) ? '-' : '@', 
                                    idx);
            }
        }
        std::cout << std::endl;
    }

private:
    static constexpr uint8_t MIN_WALL_DISTANCE = 2; // Minimum distance between walls to ensure no vertex is completely isolated
    uint8_t N, M, V, pool_size;
    std::mt19937 rng; 
    std::vector<uint8_t> pool;
    std::vector<uint8_t> parents;
    std::vector<uint8_t> wall_pool;

    // Map from pin to vertex index for quick constraint evaluation
    sp2v_t switch_p2v; // the switch's pin belong to which vertex, used to evaluate constraint C and E
    lp2v_t led_p2v; // the LED's pin belong to which vertex, used to evaluate constraint C and D
    sp2v_t switch_p2pos; // the switch's pin position in the pool, used to quickly find the complementary pin position for constraint evaluation

    // counters for print_current_configuration()
    int stats_total_trys = 0;
    int stats_total_accepted = 0;

    size_t generate_walls_obselete() {
        uint8_t min_wall_distance = 0;
        size_t tries = 0;
        // - Constraint A: Every vertex must contain 2 or more pins (no isolated or floating single pins).
        while (min_wall_distance < MIN_WALL_DISTANCE) { 
            ++tries;
            min_wall_distance = 0xFF;
            std::shuffle(wall_pool.begin(), wall_pool.end(), rng);
            std::sort(wall_pool.begin(), wall_pool.begin() + V - 1); 
            // std::cout << "Wall positions: "; for (int i = 0; i < V - 1; ++i) printf("%2d ", wall_pool[i]); std::cout << std::endl;
            for (int i = 0; i < V - 2; ++i) {
                uint8_t distance = wall_pool[i + 1] - wall_pool[i];
                min_wall_distance = std::min(min_wall_distance, distance);
            }
        }
        return tries;
    }

};
