#include "graph_parse.hpp"
#include "landmarks.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <mutex>
#include <numeric>
#include <queue>
#include <random>
#include <string>
#include <vector>
#include <future>
#include <zlib.h>

inline int get_bucket(int32_t val, int32_t last) {
    if (val == last) return 0;
    uint32_t diff = static_cast<uint32_t>(val) ^ static_cast<uint32_t>(last);
    return 32 - __builtin_clz(diff);
}

int32_t LandmarkTable::lower_bound(uint32_t u, uint32_t v) const {
    int32_t lb = 0;
    for (uint32_t l = 0; l < num_landmarks; ++l) {
        int32_t d1 = from(l, u) - from(l, v);
        int32_t d2 = to(l, v) - to(l, u);
        lb = std::max(lb, std::max(d1, d2));
    }
    return lb;
}

template <typename GraphType>
void dijkstra(const GraphType &g, uint32_t source, int metric,
              int32_t *dists, uint32_t *parents) {
    std::fill_n(dists, g.num_nodes, INF);
    if (parents) std::fill_n(parents, g.num_nodes, UINT32_MAX);
    dists[source] = 0;

    std::vector<std::pair<int32_t, uint32_t>> buckets[33];
    int32_t bucket_min[33];
    std::fill_n(bucket_min, 33, INF);

    int32_t prev = 0;
    int size = 0;

    buckets[0].push_back({0, source});
    size++;

    while (size > 0) {
        if (buckets[0].empty()) {
            int i = 1;
            while (buckets[i].empty()) i++;
            prev = bucket_min[i];
            for (const auto &p : buckets[i]) {
                int new_idx = get_bucket(p.first, prev);
                buckets[new_idx].push_back(p);
                bucket_min[new_idx] = std::min(bucket_min[new_idx], p.first);
            }
            buckets[i].clear();
            bucket_min[i] = INF;
        }

        auto [d, u] = buckets[0].back();
        buckets[0].pop_back();
        size--;

        if (d > dists[u]) continue;

        for (uint32_t i = edge_begin(g, u); i < edge_end(g, u); ++i) {
            uint32_t v = edge_target(g, i);
            int32_t weight = edge_weight(g, i, metric);
            if (dists[u] + weight < dists[v]) {
                dists[v] = dists[u] + weight;
                if (parents) parents[v] = u;
                int idx = get_bucket(dists[v], prev);
                buckets[idx].push_back({dists[v], v});
                bucket_min[idx] = std::min(bucket_min[idx], dists[v]);
                size++;
            }
        }
    }
}

uint32_t next_landmark_random(std::mt19937 &gen, uint32_t num_nodes) {
    return std::uniform_int_distribution<uint32_t>(0, num_nodes - 1)(gen);
}

uint32_t next_landmark_farthest(const LandmarkTable &lm) {
    uint32_t best = UINT32_MAX;
    int32_t best_val = -1;
    for (uint32_t v = 0; v < lm.num_nodes; ++v) {
        int32_t min_dist = INF;
        for (uint32_t l = 0; l < lm.num_landmarks; ++l)
            min_dist = std::min(min_dist, lm.to(l, v));
        if (min_dist == INF) continue;
        if (min_dist > best_val) { best_val = min_dist; best = v; }
    }
    return best;
}

template <typename GraphType>
uint32_t next_landmark_avoid(const GraphType &g, const LandmarkTable &lm,
                             int metric, std::mt19937 &gen,
                             std::vector<int32_t> &dists,
                             std::vector<uint32_t> &parents) {
    const uint32_t n = g.num_nodes;

    uint32_t root;
    if (lm.num_landmarks == 0) {
        root = next_landmark_random(gen, n);
    } else {
        std::vector<uint32_t> weights(n, 0);
        for (uint32_t v = 0; v < n; ++v) {
            int32_t min_dist = INF;
            for (uint32_t l = 0; l < lm.num_landmarks; ++l)
                min_dist = std::min(min_dist, lm.to(l, v));
            weights[v] = (min_dist < INF) ? static_cast<uint32_t>(min_dist) : 0;
        }
        std::discrete_distribution<uint32_t> dist(weights.begin(), weights.end());
        root = dist(gen);
    }

    dijkstra(g, root, metric, dists.data(), parents.data());

    std::vector<std::vector<uint32_t>> children(n);
    for (uint32_t v = 0; v < n; ++v)
        if (parents[v] != UINT32_MAX && parents[v] != v)
            children[parents[v]].push_back(v);

    std::vector<int32_t> weight(n, 0);
    for (uint32_t v = 0; v < n; ++v) {
        if (dists[v] >= INF) continue;
        weight[v] = dists[v] - lm.lower_bound(root, v);
    }

    std::vector<int64_t> size(n, 0);
    std::vector<bool> has_lm(n, false);

    for (uint32_t l = 0; l < lm.num_landmarks; ++l)
        has_lm[lm.nodes[l]] = true;

    std::vector<uint32_t> order;
    order.reserve(n);
    order.push_back(root);
    for (uint32_t i = 0; i < order.size(); ++i)
        for (uint32_t c : children[order[i]])
            order.push_back(c);

    for (int32_t i = static_cast<int32_t>(order.size()) - 1; i >= 0; --i) {
        uint32_t v = order[i];
        if (has_lm[v]) { size[v] = 0; continue; }
        size[v] = weight[v];
        for (uint32_t c : children[v]) {
            has_lm[v] = has_lm[v] || has_lm[c];
            size[v] += size[c];
        }
        if (has_lm[v]) size[v] = 0;
    }

    uint32_t w = root;
    int64_t best_size = -1;
    for (uint32_t v : order)
        if (size[v] > best_size) { best_size = size[v]; w = v; }

    uint32_t cur = w;
    while (!children[cur].empty()) {
        uint32_t next = *std::max_element(
            children[cur].begin(), children[cur].end(),
            [&](uint32_t a, uint32_t b) { return size[a] < size[b]; });
        if (size[next] == 0) break;
        cur = next;
    }
    return cur;
}

int32_t farthest_score(const LandmarkTable &lm) {
    int32_t score = -1;
    for (uint32_t v = 0; v < lm.num_nodes; ++v) {
        int32_t min_dist = INF;
        for (uint32_t l = 0; l < lm.num_landmarks; ++l)
            min_dist = std::min(min_dist, lm.to(l, v));
        if (min_dist < INF) score = std::max(score, min_dist);
    }
    return score;
}

uint32_t uncovered_arcs(const Graph &g, const LandmarkTable &landmarks,
                        const std::vector<uint32_t> &slots, int metric) {
    uint32_t count = 0;
    for (uint32_t u = 0; u < g.num_nodes; ++u)
        for (uint32_t e = edge_begin(g, u); e < edge_end(g, u); ++e) {
            uint32_t v = edge_target(g, e);
            int32_t c = edge_weight(g, e, metric);
            bool covered = false;
            for (uint32_t l : slots)
                if (c - landmarks.from(l, v) + landmarks.from(l, u) <= 0 ||
                    c - landmarks.to(l, u) + landmarks.to(l, v) <= 0) {
                    covered = true;
                    break;
                }
            if (!covered) ++count;
        }
    return count;
}

LandmarkTable build_landmarks(const Graph &g, const ReverseGraph &rg,
                              uint32_t num_lm, int metric,
                              LandmarkPolicy policy,
                              uint32_t seed) {
    LandmarkTable lm;
    lm.num_nodes = g.num_nodes;
    lm.nodes.resize(num_lm);
    lm.dist_from.resize(num_lm * g.num_nodes);
    lm.dist_to.resize(num_lm * g.num_nodes);

    std::mt19937 gen(seed);
    std::vector<int32_t> buf(g.num_nodes);
    std::vector<uint32_t> parents(g.num_nodes);

    if (policy == LandmarkPolicy::Random || policy == LandmarkPolicy::Farthest ||
        policy == LandmarkPolicy::Avoid) {

        for (uint32_t i = 0; i < num_lm; ++i) {
            uint32_t node;
            if (i == 0 || policy == LandmarkPolicy::Random)
                node = next_landmark_random(gen, g.num_nodes);
            else if (policy == LandmarkPolicy::Farthest)
                node = next_landmark_farthest(lm);
            else
                node = next_landmark_avoid(g, lm, metric, gen, buf, parents);

            lm.nodes[i] = node;

            int32_t *from_ptr = lm.dist_from.data() + i * g.num_nodes;
            int32_t *to_ptr = lm.dist_to.data() + i * g.num_nodes;
            auto fwd = std::async(std::launch::async,
                [&g, node, metric, from_ptr] { dijkstra(g, node, metric, from_ptr); });
            dijkstra(rg, node, metric, to_ptr);
            fwd.wait();

            lm.num_landmarks = i + 1;
        }
        return lm;
    }

    if (policy == LandmarkPolicy::OptimizedFarthest) {
        for (uint32_t i = 0; i < num_lm; ++i) {
            uint32_t node = (i == 0) ? next_landmark_random(gen, g.num_nodes)
                                     : next_landmark_farthest(lm);
            lm.nodes[i] = node;
            dijkstra(g, node, metric, lm.dist_from.data() + i * g.num_nodes);
            dijkstra(rg, node, metric, lm.dist_to.data() + i * g.num_nodes);
            lm.num_landmarks = i + 1;
        }

        bool any_improved = true;
        while (any_improved) {
            any_improved = false;
            for (uint32_t slot = 0; slot < num_lm; ++slot) {
                const uint32_t n = g.num_nodes;
                std::vector<int32_t> saved_from(lm.dist_from.begin() + slot * n,
                                                lm.dist_from.begin() + (slot + 1) * n);
                std::vector<int32_t> saved_to(lm.dist_to.begin() + slot * n,
                                              lm.dist_to.begin() + (slot + 1) * n);

                std::fill(lm.dist_from.begin() + slot * n,
                          lm.dist_from.begin() + (slot + 1) * n, INF);
                std::fill(lm.dist_to.begin() + slot * n,
                          lm.dist_to.begin() + (slot + 1) * n, INF);

                int32_t prev_score = farthest_score(lm);
                uint32_t candidate = next_landmark_farthest(lm);

                int32_t *from_ptr = lm.dist_from.data() + slot * n;
                int32_t *to_ptr = lm.dist_to.data() + slot * n;
                auto fwd = std::async(std::launch::async,
                    [&g, candidate, metric, from_ptr] { dijkstra(g, candidate, metric, from_ptr); });
                dijkstra(rg, candidate, metric, to_ptr);
                fwd.wait();

                if (farthest_score(lm) > prev_score) {
                    lm.nodes[slot] = candidate;
                    any_improved = true;
                } else {
                    std::copy(saved_from.begin(), saved_from.end(),
                              lm.dist_from.begin() + slot * n);
                    std::copy(saved_to.begin(), saved_to.end(),
                              lm.dist_to.begin() + slot * n);
                }
            }
        }
        return lm;
    }

    if (policy == LandmarkPolicy::MaxCover) {
        const uint32_t max_lm = 4 * num_lm;
        const uint32_t max_avoid = 5 * num_lm;

        LandmarkTable landmarks;
        landmarks.num_nodes = g.num_nodes;
        landmarks.nodes.resize(max_lm);
        landmarks.dist_from.resize(max_lm * g.num_nodes);
        landmarks.dist_to.resize(max_lm * g.num_nodes);

        auto node = next_landmark_random(gen, g.num_nodes);
        landmarks.nodes[0] = node;
        dijkstra(g, node, metric, landmarks.dist_from.data());
        dijkstra(rg, node, metric, landmarks.dist_to.data());
        landmarks.num_landmarks = 1;

        std::bernoulli_distribution cointoss(0.5);
        uint32_t avoid_calls = 1;
        while (landmarks.num_landmarks < max_lm && avoid_calls < max_avoid) {
            uint32_t kept = 0;
            for (uint32_t l = 0; l < landmarks.num_landmarks; ++l) {
                if (cointoss(gen)) {
                    if (kept != l) {
                        landmarks.nodes[kept] = landmarks.nodes[l];
                        std::copy_n(landmarks.dist_from.data() + l * g.num_nodes, g.num_nodes,
                                    landmarks.dist_from.data() + kept * g.num_nodes);
                        std::copy_n(landmarks.dist_to.data() + l * g.num_nodes, g.num_nodes,
                                    landmarks.dist_to.data() + kept * g.num_nodes);
                    }
                    ++kept;
                }
            }
            landmarks.num_landmarks = kept;

            while (landmarks.num_landmarks < num_lm && avoid_calls < max_avoid) {
                uint32_t candidate = next_landmark_avoid(g, landmarks, metric, gen, buf, parents);
                ++avoid_calls;
                bool dup = false;
                for (uint32_t l = 0; l < landmarks.num_landmarks; ++l)
                    if (landmarks.nodes[l] == candidate) { dup = true; break; }
                if (!dup) {
                    uint32_t slot = landmarks.num_landmarks;
                    int32_t *from_ptr = landmarks.dist_from.data() + slot * g.num_nodes;
                    int32_t *to_ptr = landmarks.dist_to.data() + slot * g.num_nodes;
                    auto fwd = std::async(std::launch::async,
                        [&g, candidate, metric, from_ptr] { dijkstra(g, candidate, metric, from_ptr); });
                    dijkstra(rg, candidate, metric, to_ptr);
                    fwd.wait();
                    landmarks.nodes[slot] = candidate;
                    landmarks.num_landmarks++;
                }
            }
        }

        const uint32_t num_candidates = landmarks.num_landmarks;
        const uint32_t iterations = static_cast<uint32_t>(std::log2(num_lm)) + 1;

        std::vector<uint32_t> slots_best;
        uint32_t best_cost = std::numeric_limits<uint32_t>::max();
        std::mutex best_mu;

        #pragma omp parallel for schedule(dynamic)
        for (uint32_t iter = 0; iter < iterations; ++iter) {
            std::mt19937 local_gen(seed + iter);
            std::vector<uint32_t> local_indices(num_candidates);
            std::iota(local_indices.begin(), local_indices.end(), 0);
            std::shuffle(local_indices.begin(), local_indices.end(), local_gen);

            std::vector<uint32_t> slots_curr(num_lm);
            std::copy_n(local_indices.begin(), num_lm, slots_curr.begin());

            bool improved = true;
            while (improved) {
                improved = false;
                uint32_t cur_cost = uncovered_arcs(g, landmarks, slots_curr, metric);

                std::vector<bool> in_S(num_candidates, false);
                for (uint32_t s : slots_curr) in_S[s] = true;

                struct Swap { uint32_t si, pi, profit; };
                std::vector<Swap> improving;

                for (uint32_t pi = 0; pi < num_candidates; ++pi) {
                    if (in_S[pi]) continue;
                    for (uint32_t si = 0; si < num_lm; ++si) {
                        std::vector<uint32_t> test = slots_curr;
                        test[si] = pi;
                        uint32_t test_cost = uncovered_arcs(g, landmarks, test, metric);
                        if (test_cost < cur_cost)
                            improving.push_back({si, pi, cur_cost - test_cost});
                    }
                }

                if (!improving.empty()) {
                    std::vector<uint32_t> weights;
                    for (auto &sw : improving) weights.push_back(sw.profit);
                    std::discrete_distribution<uint32_t> wdist(weights.begin(), weights.end());
                    auto &chosen = improving[wdist(local_gen)];
                    slots_curr[chosen.si] = chosen.pi;
                    improved = true;
                }
            }

            uint32_t local_cost = uncovered_arcs(g, landmarks, slots_curr, metric);
            {
                std::lock_guard<std::mutex> lk(best_mu);
                if (local_cost < best_cost) {
                    best_cost = local_cost;
                    slots_best = slots_curr;
                }
            }
        }

        for (uint32_t i = 0; i < num_lm; ++i) {
            uint32_t src = slots_best[i];
            lm.nodes[i] = landmarks.nodes[src];
            std::copy_n(landmarks.dist_from.data() + src * g.num_nodes, g.num_nodes,
                        lm.dist_from.data() + i * g.num_nodes);
            std::copy_n(landmarks.dist_to.data() + src * g.num_nodes, g.num_nodes,
                        lm.dist_to.data() + i * g.num_nodes);
        }
        lm.num_landmarks = num_lm;
        return lm;
    }
    __builtin_unreachable();
}

static void gz_write_all(gzFile f, const void *buf, size_t len) {
    const char *p = static_cast<const char *>(buf);
    while (len > 0) {
        unsigned chunk = static_cast<unsigned>(std::min(len, size_t(1u << 30)));
        gzwrite(f, p, chunk);
        p += chunk;
        len -= chunk;
    }
}

static void gz_read_all(gzFile f, void *buf, size_t len) {
    char *p = static_cast<char *>(buf);
    while (len > 0) {
        unsigned chunk = static_cast<unsigned>(std::min(len, size_t(1u << 30)));
        gzread(f, p, chunk);
        p += chunk;
        len -= chunk;
    }
}

void save_landmarks(const LandmarkTable &t, const std::string &path) {
    gzFile out = gzopen(path.c_str(), "wb1");
    gz_write_all(out, &t.num_landmarks, sizeof(t.num_landmarks));
    gz_write_all(out, &t.num_nodes, sizeof(t.num_nodes));
    gz_write_all(out, t.nodes.data(), t.nodes.size() * sizeof(uint32_t));
    gz_write_all(out, t.dist_from.data(), t.dist_from.size() * sizeof(int32_t));
    gz_write_all(out, t.dist_to.data(), t.dist_to.size() * sizeof(int32_t));
    gzclose(out);
}

LandmarkTable load_landmarks(const std::string &path) {
    gzFile in = gzopen(path.c_str(), "rb");
    LandmarkTable t;
    gz_read_all(in, &t.num_landmarks, sizeof(t.num_landmarks));
    gz_read_all(in, &t.num_nodes, sizeof(t.num_nodes));
    t.nodes.resize(t.num_landmarks);
    t.dist_from.resize(t.num_landmarks * t.num_nodes);
    t.dist_to.resize(t.num_landmarks * t.num_nodes);
    gz_read_all(in, t.nodes.data(), t.nodes.size() * sizeof(uint32_t));
    gz_read_all(in, t.dist_from.data(), t.dist_from.size() * sizeof(int32_t));
    gz_read_all(in, t.dist_to.data(), t.dist_to.size() * sizeof(int32_t));
    gzclose(in);
    return t;
}