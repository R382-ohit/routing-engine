# High-Performance Network Routing Engine

A standalone, optimized network pathfinding engine written in modern C++ engineered for dynamic logistics grids and real-time transit telemetry.

## Technical Highlights
* **Algorithm:** Hybrid Optimized Dijkstra / $A^*$ implementation utilizing a min-priority queue memory structure.
* **Dynamic Time-Variant Costs:** Evaluates path cost using the relation $f(x) = \text{Distance} \times \text{Traffic Factor}$, allowing for runtime weight adjustments without re-allocating memory blocks.
* **Complexity:** Time complexity bound to $\mathcal{O}((E + V) \log V)$ using an adjacency-list backed priority min-heap queue configuration.

## How to Build & Run
```bash
# Compile the engine
g++ -O3 main.cpp -o routing_engine

# Run the simulation executable
./routing_engine