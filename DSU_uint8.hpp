#pragma once
#include <vector>
#include <numeric>
// Disjoint Set Union (Union-Find) for uint8_t
struct DSU_uint8 {
    std::vector<uint8_t> parent;
    DSU_uint8(uint8_t n) {
        parent.resize(n);
        std::iota(parent.begin(), parent.end(), 0);
    }
    DSU_uint8(std::vector<uint8_t> p) : parent(std::move(p)) {}
    uint8_t find(uint8_t i) {
        if (parent[i] == i) return i;
        return parent[i] = find(parent[i]);
    }
    void unite(uint8_t i, uint8_t j) {
        uint8_t root_i = find(i);
        uint8_t root_j = find(j);
        if (root_i != root_j) {
            parent[root_i] = root_j;
        }
    }
};