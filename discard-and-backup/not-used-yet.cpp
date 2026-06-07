// Helper DFS to spot series paths
bool has_path(int current, int target, const std::map<int, std::vector<int>>& adj, std::set<int>& visited) {
    if (current == target) return true;
    if (visited.count(current)) return false;
    visited.insert(current);
    if (adj.count(current)) {
        for (int next_node : adj.at(current)) {
            if (has_path(next_node, target, adj, visited)) return true;
        }
    }
    return false;
}

// Evaluates unique LED bitmask outputs. Returns -1 if an illegal circuit configuration happens.
int evaluate_circuit(const Circuit& circuit, int N, int M, int V) {
    int total_pins = circuit.elements.size();
    std::map<uint32_t, int> pin_to_vertex;
    
    size_t last_wall = 0;
    for (int i = 0; i < V; ++i) {
        size_t next_wall = (i < V - 1) ? circuit.wall_positions[i] : total_pins;
        for (size_t j = last_wall; j < next_wall; ++j) {
            pin_to_vertex[circuit.elements[j]] = i;
        }
        last_wall = next_wall;
    }

    uint32_t p_pos = encode_pin(POWER, POS, 0);
    uint32_t p_neg = encode_pin(POWER, NEG, 0);
    
    std::set<int> unique_led_combinations;
    int states_count = 1 << N;

    for (int mask = 0; mask < states_count; ++mask) {
        DSU dsu(V);

        for (int i = 0; i < N; ++i) {
            int sw_state = (mask >> i) & 1;
            uint32_t pin0 = encode_pin(i, SWITCH, 0);
            uint32_t target_pin = encode_pin(i, SWITCH, sw_state == 0 ? 1 : 2);
            dsu.unite(pin_to_vertex[pin0], pin_to_vertex[target_pin]);
        }

        // Step 2.3: Check Short Circuit
        int pos_root = dsu.find(pin_to_vertex[p_pos]);
        int neg_root = dsu.find(pin_to_vertex[p_neg]);
        if (pos_root == neg_root) {
            return -1; 
        }

        int current_led_mask = 0;
        std::map<int, std::vector<int>> indirect_graph;

        // Step 2.4: Evaluate direct pins
        for (int i = 0; i < M; ++i) {
            int led_pos_root = dsu.find(pin_to_vertex[encode_pin(i, LED, 0)]);
            int led_neg_root = dsu.find(pin_to_vertex[encode_pin(i, LED, 1)]);

            if (led_pos_root == pos_root && led_neg_root == neg_root) {
                current_led_mask |= (1 << i); // ON
            } else if (led_pos_root == neg_root || led_neg_root == pos_root || led_pos_root == led_neg_root) {
                // OFF (Reverse biased or grounded natively)
            } else {
                // Step 2.5: Complex/Indirect Topology evaluation
                indirect_graph[led_pos_root].push_back(led_neg_root);
            }
        }

        // Check if any leftover elements build an active sequence from Power + to Power -
        std::set<int> visited;
        if (has_path(pos_root, neg_root, indirect_graph, visited)) {
            return -1; // Dim/Murky/Series detected. Scrap entire circuit layout.
        }

        unique_led_combinations.insert(current_led_mask);
    }

    return unique_led_combinations.size();
}