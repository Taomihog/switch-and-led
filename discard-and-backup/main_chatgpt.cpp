#include <algorithm>
#include <array>
#include <bitset>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// ============================================================
// Bit Encoding Utilities
// ============================================================

enum class ComponentType : uint32_t {
    Power  = 0,
    LED    = 1,
    Switch = 2
};

enum class PowerPin : uint32_t {
    Plus  = 0,
    Minus = 1
};

enum class LEDPin : uint32_t {
    Plus  = 0,
    Minus = 1
};

enum class SwitchPin : uint32_t {
    Pin0 = 0,
    Pin1 = 1,
    Pin2 = 2
};

constexpr uint32_t make_pin(ComponentType type,
                            uint32_t componentIndex,
                            uint32_t pinId)
{
    return ((componentIndex & 0x0FFFFFFFu) << 4u) |
           ((static_cast<uint32_t>(type) & 0x3u) << 2u) |
           (pinId & 0x3u);
}

constexpr uint32_t pin_id(uint32_t encoded)
{
    return encoded & 0x3u;
}

constexpr ComponentType component_type(uint32_t encoded)
{
    return static_cast<ComponentType>((encoded >> 2u) & 0x3u);
}

constexpr uint32_t component_index(uint32_t encoded)
{
    return encoded >> 4u;
}

// ============================================================
// Circuit Structures
// ============================================================

struct Circuit {
    std::vector<uint32_t> elements;
    std::vector<size_t> wall_positions;
};

struct EvaluatedCircuit {
    Circuit circuit;
    size_t playablePatterns = 0;
    std::set<uint32_t> uniqueLedPatterns;
};

struct LEDInfo {
    int plusVertex  = -1;
    int minusVertex = -1;
};

struct SwitchInfo {
    int pin0Vertex = -1;
    int pin1Vertex = -1;
    int pin2Vertex = -1;
};

// ============================================================
// Disjoint Set Union
// ============================================================

class DisjointSet {
public:
    explicit DisjointSet(size_t n)
        : parent_(n), rank_(n, 0)
    {
        for (size_t i = 0; i < n; ++i) {
            parent_[i] = i;
        }
    }

    size_t find(size_t x)
    {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]);
        }
        return parent_[x];
    }

    void unite(size_t a, size_t b)
    {
        a = find(a);
        b = find(b);

        if (a == b) {
            return;
        }

        if (rank_[a] < rank_[b]) {
            std::swap(a, b);
        }

        parent_[b] = a;

        if (rank_[a] == rank_[b]) {
            rank_[a]++;
        }
    }

private:
    std::vector<size_t> parent_;
    std::vector<size_t> rank_;
};

// ============================================================
// Circuit Generator
// ============================================================

class CircuitGenerator {
public:
    CircuitGenerator(size_t numSwitches,
                     size_t numLEDs,
                     size_t numVertices)
        : N_(numSwitches),
          M_(numLEDs),
          V_(numVertices),
          rng_(std::random_device{}())
    {
        build_all_pins();
    }

    Circuit generate()
    {
        constexpr int MAX_REPARTITION_ATTEMPTS = 100;

        while (true) {

            std::vector<uint32_t> shuffled = allPins_;
            std::shuffle(shuffled.begin(), shuffled.end(), rng_);

            for (int attempt = 0;
                 attempt < MAX_REPARTITION_ATTEMPTS;
                 ++attempt)
            {
                auto maybeCircuit = try_partition(shuffled);

                if (maybeCircuit.has_value()) {
                    return *maybeCircuit;
                }
            }
        }
    }

private:
    size_t N_;
    size_t M_;
    size_t V_;

    std::vector<uint32_t> allPins_;
    std::mt19937 rng_;

private:
    void build_all_pins()
    {
        allPins_.clear();

        // Power source
        allPins_.push_back(
            make_pin(ComponentType::Power, 0,
                     static_cast<uint32_t>(PowerPin::Plus)));

        allPins_.push_back(
            make_pin(ComponentType::Power, 0,
                     static_cast<uint32_t>(PowerPin::Minus)));

        // LEDs
        for (uint32_t i = 0; i < M_; ++i) {
            allPins_.push_back(
                make_pin(ComponentType::LED, i,
                         static_cast<uint32_t>(LEDPin::Plus)));

            allPins_.push_back(
                make_pin(ComponentType::LED, i,
                         static_cast<uint32_t>(LEDPin::Minus)));
        }

        // Switches
        for (uint32_t i = 0; i < N_; ++i) {
            allPins_.push_back(
                make_pin(ComponentType::Switch, i,
                         static_cast<uint32_t>(SwitchPin::Pin0)));

            allPins_.push_back(
                make_pin(ComponentType::Switch, i,
                         static_cast<uint32_t>(SwitchPin::Pin1)));

            allPins_.push_back(
                make_pin(ComponentType::Switch, i,
                         static_cast<uint32_t>(SwitchPin::Pin2)));
        }
    }

    std::optional<Circuit>
    try_partition(const std::vector<uint32_t>& shuffled)
    {
        const size_t totalPins = shuffled.size();

        if (V_ * 2 > totalPins) {
            return std::nullopt;
        }

        // Partition sizes >= 2
        std::vector<size_t> sizes(V_, 2);
        size_t remaining = totalPins - 2 * V_;

        std::uniform_int_distribution<size_t> dist(0, V_ - 1);

        while (remaining > 0) {
            sizes[dist(rng_)]++;
            remaining--;
        }

        std::vector<size_t> walls;
        size_t running = 0;

        for (size_t i = 0; i < V_ - 1; ++i) {
            running += sizes[i];
            walls.push_back(running);
        }

        Circuit c;
        c.elements = shuffled;
        c.wall_positions = walls;

        if (validate(c)) {
            return c;
        }

        return std::nullopt;
    }

    bool validate(const Circuit& c)
    {
        auto vertexMap = build_vertex_map(c);

        int powerPlusVertex  = -1;
        int powerMinusVertex = -1;

        for (const auto& [pin, v] : vertexMap) {

            if (component_type(pin) == ComponentType::Power) {

                if (pin_id(pin) == 0) {
                    powerPlusVertex = v;
                }
                else {
                    powerMinusVertex = v;
                }
            }
        }

        // Constraint B
        if (powerPlusVertex == powerMinusVertex) {
            return false;
        }

        // Constraint C, D, E
        for (const auto& [pin, v] : vertexMap) {

            auto type = component_type(pin);
            auto idx  = component_index(pin);

            if (type == ComponentType::LED) {

                uint32_t other =
                    make_pin(ComponentType::LED,
                             idx,
                             pin_id(pin) == 0 ? 1 : 0);

                int ov = vertexMap.at(other);

                // Constraint C
                if (v == ov) {
                    return false;
                }

                // Constraint D
                bool directPowerConnection =
                    ((v == powerPlusVertex && ov == powerMinusVertex) ||
                     (v == powerMinusVertex && ov == powerPlusVertex));

                if (directPowerConnection) {
                    return false;
                }
            }

            if (type == ComponentType::Switch &&
                pin_id(pin) == 0)
            {
                uint32_t p1 =
                    make_pin(ComponentType::Switch, idx, 1);

                uint32_t p2 =
                    make_pin(ComponentType::Switch, idx, 2);

                int v1 = vertexMap.at(p1);
                int v2 = vertexMap.at(p2);

                // Constraint C
                if (v == v1 || v == v2) {
                    return false;
                }

                // Constraint E
                bool short1 =
                    ((v == powerPlusVertex &&
                      v1 == powerMinusVertex) ||
                     (v == powerMinusVertex &&
                      v1 == powerPlusVertex));

                bool short2 =
                    ((v == powerPlusVertex &&
                      v2 == powerMinusVertex) ||
                     (v == powerMinusVertex &&
                      v2 == powerPlusVertex));

                if (short1 || short2) {
                    return false;
                }
            }
        }

        return true;
    }

    std::unordered_map<uint32_t, int>
    build_vertex_map(const Circuit& c)
    {
        std::unordered_map<uint32_t, int> result;

        size_t currentVertex = 0;
        size_t wallIndex     = 0;

        for (size_t i = 0; i < c.elements.size(); ++i) {

            if (wallIndex < c.wall_positions.size() &&
                i == c.wall_positions[wallIndex])
            {
                currentVertex++;
                wallIndex++;
            }

            result[c.elements[i]] =
                static_cast<int>(currentVertex);
        }

        return result;
    }
};

// ============================================================
// Circuit Evaluator
// ============================================================

class CircuitEvaluator {
public:
    CircuitEvaluator(size_t numSwitches,
                     size_t numLEDs,
                     size_t numVertices)
        : N_(numSwitches),
          M_(numLEDs),
          V_(numVertices)
    {
    }

    std::optional<EvaluatedCircuit>
    evaluate(const Circuit& circuit)
    {
        auto vertexMap = build_vertex_map(circuit);

        auto leds     = build_led_info(vertexMap);
        auto switches = build_switch_info(vertexMap);

        int powerPlusVertex =
            vertexMap.at(
                make_pin(ComponentType::Power, 0, 0));

        int powerMinusVertex =
            vertexMap.at(
                make_pin(ComponentType::Power, 0, 1));

        EvaluatedCircuit result;
        result.circuit = circuit;

        const uint32_t totalStates = 1u << N_;

        for (uint32_t switchMask = 0;
             switchMask < totalStates;
             ++switchMask)
        {
            DisjointSet dsu(V_);

            // Apply switches
            for (size_t sw = 0; sw < N_; ++sw) {

                bool state = (switchMask >> sw) & 1u;

                if (!state) {
                    dsu.unite(
                        switches[sw].pin0Vertex,
                        switches[sw].pin1Vertex);
                }
                else {
                    dsu.unite(
                        switches[sw].pin0Vertex,
                        switches[sw].pin2Vertex);
                }
            }

            auto pPlusRoot =
                dsu.find(powerPlusVertex);

            auto pMinusRoot =
                dsu.find(powerMinusVertex);

            // Short circuit
            if (pPlusRoot == pMinusRoot) {
                return std::nullopt;
            }

            uint32_t ledPattern = 0;

            enum class LEDState {
                Unknown,
                On,
                Off
            };

            std::vector<LEDState> states(
                M_, LEDState::Unknown);

            // Direct evaluation
            for (size_t i = 0; i < M_; ++i) {

                auto lp =
                    dsu.find(leds[i].plusVertex);

                auto lm =
                    dsu.find(leds[i].minusVertex);

                if (lp == lm) {
                    states[i] = LEDState::Off;
                    continue;
                }

                if (lp == pPlusRoot &&
                    lm == pMinusRoot)
                {
                    states[i] = LEDState::On;
                    ledPattern |= (1u << i);
                    continue;
                }

                if (lp == pMinusRoot ||
                    lm == pPlusRoot)
                {
                    states[i] = LEDState::Off;
                    continue;
                }
            }

            // Build graph for unresolved LEDs
            using Graph =
                std::unordered_map<size_t,
                                   std::vector<size_t>>;

            Graph graph;

            for (size_t i = 0; i < M_; ++i) {

                if (states[i] != LEDState::Unknown) {
                    continue;
                }

                size_t lp =
                    dsu.find(leds[i].plusVertex);

                size_t lm =
                    dsu.find(leds[i].minusVertex);

                graph[lp].push_back(lm);
            }

            // BFS from power+
            std::unordered_set<size_t> reachable;
            std::queue<size_t> q;

            q.push(pPlusRoot);
            reachable.insert(pPlusRoot);

            while (!q.empty()) {

                size_t u = q.front();
                q.pop();

                auto it = graph.find(u);

                if (it == graph.end()) {
                    continue;
                }

                for (size_t v : it->second) {

                    if (!reachable.count(v)) {
                        reachable.insert(v);
                        q.push(v);
                    }
                }
            }

            // Invalid series topology
            if (reachable.count(pMinusRoot)) {
                return std::nullopt;
            }

            // Remaining LEDs OFF
            result.uniqueLedPatterns.insert(ledPattern);
        }

        result.playablePatterns =
            result.uniqueLedPatterns.size();

        return result;
    }

private:
    size_t N_;
    size_t M_;
    size_t V_;

private:
    std::unordered_map<uint32_t, int>
    build_vertex_map(const Circuit& c)
    {
        std::unordered_map<uint32_t, int> result;

        size_t currentVertex = 0;
        size_t wallIndex     = 0;

        for (size_t i = 0; i < c.elements.size(); ++i) {

            if (wallIndex < c.wall_positions.size() &&
                i == c.wall_positions[wallIndex])
            {
                currentVertex++;
                wallIndex++;
            }

            result[c.elements[i]] =
                static_cast<int>(currentVertex);
        }

        return result;
    }

    std::vector<LEDInfo>
    build_led_info(
        const std::unordered_map<uint32_t, int>& map)
    {
        std::vector<LEDInfo> leds(M_);

        for (size_t i = 0; i < M_; ++i) {

            leds[i].plusVertex =
                map.at(make_pin(ComponentType::LED,
                                i,
                                0));

            leds[i].minusVertex =
                map.at(make_pin(ComponentType::LED,
                                i,
                                1));
        }

        return leds;
    }

    std::vector<SwitchInfo>
    build_switch_info(
        const std::unordered_map<uint32_t, int>& map)
    {
        std::vector<SwitchInfo> switches(N_);

        for (size_t i = 0; i < N_; ++i) {

            switches[i].pin0Vertex =
                map.at(make_pin(ComponentType::Switch,
                                i,
                                0));

            switches[i].pin1Vertex =
                map.at(make_pin(ComponentType::Switch,
                                i,
                                1));

            switches[i].pin2Vertex =
                map.at(make_pin(ComponentType::Switch,
                                i,
                                2));
        }

        return switches;
    }
};

// ============================================================
// Pretty Printing
// ============================================================

std::string pin_to_string(uint32_t p)
{
    auto type = component_type(p);
    auto idx  = component_index(p);
    auto pin  = pin_id(p);

    switch (type) {

    case ComponentType::Power:
        return pin == 0 ? "PWR+" : "PWR-";

    case ComponentType::LED:
        return "LED" + std::to_string(idx) +
               (pin == 0 ? "+" : "-");

    case ComponentType::Switch:
        return "SW" + std::to_string(idx) +
               ".P" + std::to_string(pin);
    }

    return "?";
}

void print_circuit(const Circuit& c)
{
    size_t currentVertex = 0;
    size_t wallIndex     = 0;

    std::cout << "\nCircuit Vertices:\n";

    for (size_t i = 0; i < c.elements.size(); ++i) {

        if (wallIndex < c.wall_positions.size() &&
            i == c.wall_positions[wallIndex])
        {
            std::cout << "\n";
            currentVertex++;
            wallIndex++;
        }

        std::cout << "[V" << currentVertex << "] "
                  << std::setw(8)
                  << pin_to_string(c.elements[i])
                  << "   ";
    }

    std::cout << "\n";
}

// ============================================================
// Main
// ============================================================

int main()
{
    constexpr size_t N = 3;
    constexpr size_t M = 4;
    constexpr size_t V = 6;

    constexpr size_t MONTE_CARLO_ITERATIONS = 10000;

    CircuitGenerator generator(N, M, V);
    CircuitEvaluator evaluator(N, M, V);

    size_t bestScore = 0;
    std::optional<EvaluatedCircuit> bestCircuit;

    for (size_t iter = 0;
         iter < MONTE_CARLO_ITERATIONS;
         ++iter)
    {
        Circuit c = generator.generate();

        auto evaluated = evaluator.evaluate(c);

        if (!evaluated.has_value()) {
            continue;
        }

        if (evaluated->playablePatterns >
            bestScore)
        {
            bestScore =
                evaluated->playablePatterns;

            bestCircuit = evaluated;
        }
    }

    if (!bestCircuit.has_value()) {

        std::cout
            << "No valid circuit found.\n";

        return 0;
    }

    std::cout
        << "====================================\n";

    std::cout
        << "BEST CIRCUIT FOUND\n";

    std::cout
        << "====================================\n";

    std::cout
        << "Unique LED patterns: "
        << bestCircuit->playablePatterns
        << "\n";

    print_circuit(bestCircuit->circuit);

    std::cout << "\nPlayable LED Patterns:\n";

    for (uint32_t pattern :
         bestCircuit->uniqueLedPatterns)
    {
        std::cout << "  ";

        for (int i = static_cast<int>(M) - 1;
             i >= 0;
             --i)
        {
            std::cout
                << (((pattern >> i) & 1u)
                        ? '1'
                        : '0');
        }

        std::cout << "\n";
    }

    return 0;
}