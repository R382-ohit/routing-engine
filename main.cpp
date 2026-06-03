#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>

// Represents a geographical coordinate for heuristic calculations (A*)
struct Node {
    int id;
    double latitude;
    double longitude;
};

// Represents a directed edge with traffic/distance metrics
struct Edge {
    int target_id;
    double distance;      // Base distance in km
    double traffic_factor; // 1.0 = clear, >1.0 = congested
    
    double get_weight() const {
        return distance * traffic_factor; // Total routing cost
    }
};

class RoutingEngine {
private:
    std::unordered_map<int, Node> nodes;
    std::unordered_map<int, std::vector<Edge>> adjacency_list;

    // Haversine-inspired heuristic for A* (Straight-line distance estimation)
    double calculate_heuristic(int current_id, int target_id) {
        if (nodes.find(current_id) == nodes.end() || nodes.find(target_id) == nodes.end()) {
            return 0.0;
        }
        auto n1 = nodes[current_id];
        auto n2 = nodes[target_id];
        return std::sqrt(std::pow(n1.latitude - n2.latitude, 2) + std::pow(n1.longitude - n2.longitude, 2)) * 111.0; 
    }

public:
    void add_node(int id, double lat, double lon) {
        nodes[id] = {id, lat, lon};
    }

    void add_edge(int source, int target, double distance, double traffic_factor = 1.0) {
        adjacency_list[source].push_back({target, distance, traffic_factor});
    }

    // High-performance A* / Optimized Dijkstra Pathfinder
    std::pair<double, std::vector<int>> find_optimal_path(int source, int destination) {
        // Min-Heap tracking pair: <Total Estimated Cost (f_score), Current Node ID>
        std::priority_queue<std::pair<double, int>, 
                            std::vector<std::pair<double, int>>, 
                            std::greater<std::pair<double, int>>> pq;

        std::unordered_map<int, double> g_score; // Actual cost from source to node
        std::unordered_map<int, int> parent;     // Reconstruct path

        for (const auto& pair : nodes) {
            g_score[pair.first] = std::numeric_limits<double>::infinity();
        }

        g_score[source] = 0.0;
        pq.push({calculate_heuristic(source, destination), source});

        while (!pq.empty()) {
            
            auto top_element = pq.top();
            // double current_f = top_element.first;
            int current_id = top_element.second;
            pq.pop();

            if (current_id == destination) {
                break; // Target reached
            }

            if (adjacency_list.find(current_id) == adjacency_list.end()) continue;

            for (const auto& edge : adjacency_list[current_id]) {
                double tentative_g = g_score[current_id] + edge.get_weight();

                if (tentative_g < g_score[edge.target_id]) {
                    g_score[edge.target_id] = tentative_g;
                    parent[edge.target_id] = current_id;
                    double f_score = tentative_g + calculate_heuristic(edge.target_id, destination);
                    pq.push({f_score, edge.target_id});
                }
            }
        }

        // Path reconstruction
        if (g_score[destination] == std::numeric_limits<double>::infinity()) {
            return { -1.0, {} }; // Path not found
        }

        std::vector<int> path;
        int curr = destination;
        while (curr != source) {
            path.push_back(curr);
            curr = parent[curr];
        }
        path.push_back(source);
        std::reverse(path.begin(), path.end());

        return { g_score[destination], path };
    }
};

int main() {
    RoutingEngine engine;

    // Build a sample logistics grid
    engine.add_node(1, 22.5726, 88.3639); // Node 1
    engine.add_node(2, 22.5800, 88.3700); // Node 2
    engine.add_node(3, 22.5900, 88.3600); // Node 3
    engine.add_node(4, 22.6000, 88.3800); // Node 4

    // Interconnect network with variable traffic costs
    engine.add_edge(1, 2, 5.0, 1.2);  // 5km, slight congestion
    engine.add_edge(1, 3, 12.0, 1.0); // 12km, clear route
    engine.add_edge(2, 4, 6.0, 3.0);  // 6km, heavy traffic bottleneck
    engine.add_edge(3, 4, 4.0, 1.0);  // 4km, clear route

    int start = 1, end = 4;
    
    
    auto result = engine.find_optimal_path(start, end);
    double cost = result.first;
    std::vector<int> path = result.second;

    if (cost != -1.0) {
        std::cout << "Optimal Route Cost (Time/Distance equivalent): " << cost << " units\n";
        std::cout << "Path Sequence: ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << path[i] << (i == path.size() - 1 ? "" : " -> ");
        }
        std::cout << "\n";
    } else {
        std::cout << "No viable path found within the network configuration.\n";
    }

    return 0;
}