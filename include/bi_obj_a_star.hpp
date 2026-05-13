#include "landmarks.hpp"
#include "graph_parse.hpp"
#include <vector>
#include <queue>
#include <limits>
#include <memory>

struct Node {
    uint32_t state;
    int32_t g1, g2;
    int32_t f1, f2;
    std::shared_ptr<Node> parent;

    bool operator>(const Node& other) const {
        if (f1 != other.f1) return f1 > other.f1;
        return f2 > other.f2; 
    }
};

class BOBALT {
private:
    const Graph &graph;
    const std::vector<LandmarkTable> &landmarks;

    int32_t h1(uint32_t u, uint32_t target) const { return 0; }
    int32_t h2(uint32_t u, uint32_t target) const { return 0; }
    int32_t ub1(uint32_t u, uint32_t target) const { return std::numeric_limits<int32_t>::max(); }
    int32_t ub2(uint32_t u, uint32_t target) const { return std::numeric_limits<int32_t>::max(); }

public:

        BOBALT(const Graph &g, const std::vector<LandmarkTable> &lm) : graph(g), landmarks(lm) {}
    
    std::vector<Node> query(uint32_t source, uint32_t target) {
        const int32_t INF = std::numeric_limits<int32_t>::max();

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        std::vector<Node> sol;

        std::vector<int32_t> g1_min(graph.num_nodes, INF);
        std::vector<int32_t> g2_min(graph.num_nodes, INF);
        std::vector<int32_t> h_prime_1(graph.num_nodes, -1); 

        Node start_node;
        start_node.state = source;
        start_node.g1 = 0;
        start_node.g2 = 0;
        start_node.f1 = h1(source, target);
        start_node.f2 = h2(source, target);
        start_node.parent = nullptr;
        
        open.push(start_node);

        while (!open.empty()) {
            
            Node x = open.top();
            open.pop();
            uint32_t s_x = x.state;

            if (x.f1 >= g1_min[source]) break;
            if (x.g2 >= g2_min[s_x] || x.f2 >= g2_min[target]) continue;
            if (g2_min[s_x] == INF) {
                h_prime_1[s_x] = x.g1;
            }

            g2_min[s_x] = x.g2;

            if (s_x == target) {
                if (!sol.empty() && sol.back().f1 == x.f1) {
                    sol.pop_back(); 
                }
                sol.push_back(x); 
                continue;
            }

            if (x.g2 + ub2(s_x, target) < g2_min[target]) {
                g2_min[target] = x.g2 + ub2(s_x, target);
                if (!sol.empty() && sol.back().f1 == x.f1) {
                    sol.pop_back();
                }
                sol.push_back(x); 
                if (h1(s_x, target) == ub1(s_x, target)) continue;
            }

            for (uint32_t e = graph.offset[s_x]; e < graph.offset[s_x + 1]; ++e) {
                uint32_t t = graph.target[e];
                Node y;
                y.state = t;
                y.g1 = x.g1 + graph.distance[e];
                y.g2 = x.g2 + graph.travel_time[e]; 
                y.f1 = y.g1 + h1(t, target);
                y.f2 = y.g2 + h2(t, target);
                y.parent = x.state;
                if (y.g2 >= g2_min[t] || y.f2 >= g2_min[target]) continue;
                if (y.f1 >= g1_min[source]) continue;
                open.push(y);
            }
        }

        return sol; 
    }
};
