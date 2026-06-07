```markdown
You are an expert C++ developer. Implement a Monte Carlo simulation engine to find optimal black-box circuit configurations involving N switches, M LEDs, and 1 DC power source.

### Data Encoding Specification
Represent every electrical node/pin as a single `uint32_t` integer using explicit bit-masking:
- Bits 0-1: Pin ID (For Power: 0=+, 1=-. For LED: 0=+, 1=-. For Switch: 0=Pin 0, 1=Pin 1, 2=Pin 2)
- Bits 2-3: Component Type (0 = Power, 1 = LED, 2 = Switch)
- Bits 4-31: Component Index (0 to N-1 or M-1)

### Component Definitions
- Switches: Single Pole Double Throw (SPDT). Has State 0 (Pin 0 connects to Pin 1) and State 1 (Pin 0 connects to Pin 2). State 'x' is ignored.
- LEDs: Directional (+ to -). Lit if current flows correctly from + to -; blocks reverse current.
- Power Source: 1 positive (+) pole, 1 negative (-) pole.

### Step 1: Circuit Generation (Class CircuitGenerator)
Generate random circuits consisting of 'V' vertices.
1. Collect all pins into a flat array of size (2 + 2M + 3N).
2. Generate a random permutation of this array.
3. Randomly partition this array into V groups by placing V-1 walls. Each group represents a vertex (an electrical junction where all contained pins are hard-wired together).
4. Store this in a structure:
struct Circuit {
    std::vector<uint32_t> elements;
    std::vector<size_t> wall_positions; // Indices where a new vertex group begins
};

Enforce these structural sanity constraints. If violated, re-partition. If re-partitioning fails 100 times, restart from step 1 (re-shuffling):
- Constraint A: Every vertex must contain 2 or more pins (no isolated or floating single pins).
- Constraint B: A vertex cannot contain BOTH the Power (+) and Power (-) poles (instant hard short circuit).
- Constraint C: The same LED's + and - pins, or a switch's Pin 0 and Pin 1/2, cannot be inside the same vertex (renders the component natively useless).
- Constraint D: An LED's + and - pins cannot span directly between the Power (+) vertex and Power (-) vertex. (This would render the LED permanently on or permanently reverse-biased/off, bypassing the switch matrix).
- Constraint E: A switch's Pin 0 and either Pin 1 or Pin 2 cannot span directly between the Power (+) vertex and Power (-) vertex (flipping this switch would cause a guaranteed short circuit).

### Step 2: Evaluate LED States across all 2^N Switch Combinations
For a generated circuit, evaluate its state across all 2^N switch combinations. Use a Disjoint-Set (Union-Find) data structure to evaluate connectivity for each combination:

1. Initialize Union-Find with the V structural vertices.
2. For the current switch configuration bitmask, iterate through all N switches:
   - If switch state bit is 0: Union the vertex containing Pin 0 with the vertex containing Pin 1.
   - If switch state bit is 1: Union the vertex containing Pin 0 with the vertex containing Pin 2.
3. Check Short Circuit: If the vertex containing Power (+) and the vertex containing Power (-) end up in the same Union-Find set, discard this entire circuit configuration immediately. Restart at Step 1.
4. Evaluate direct LED states using the Union-Find sets:
   - Find the root representative element for the Power (+) vertex and Power (-) vertex.
   - For each LED:
     - LED is ON if its (+) pin is in the Power (+) set AND its (-) pin is in the Power (-) set.
     - LED is OFF if its (+) pin is in the Power (-) set OR its (-) pin is in the Power (+) set (reverse biased).
     - LED is OFF if both pins belong to the exact same Union-Find set.
5. Evaluate indirect/complex topologies:
   For any LEDs whose states are still undetermined after step 4, construct an explicit directed graph where the nodes are the Union-Find root sets, and directed edges represent the remaining LEDs pointing from their (+) set to their (-) set.
   - An LED is OFF if its nodes cannot trace a path back to both a Power (+) source and Power (-) sink.
   - If an LED path successfully traces from the Power (+) set through any remaining LEDs to the Power (-) set, this constitutes an invalid complex/series topology. Discard this entire circuit configuration immediately and restart at Step 1.

### Requirements
1. Implement `CircuitGenerator` and the evaluation engine in clean, modern C++ (C++17 or later).
2. Write a `main()` function setting N=3 switches, M=4 LEDs, V=6 vertices. Run a Monte Carlo pass of 10,000 iterations to find a valid circuit maximizing the number of unique, playable LED combinations, and print the best configuration found.

```