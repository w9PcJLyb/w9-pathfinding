#include "include/resumable_space_time_dijkstra.h"


ResumableSpaceTimeDijkstra::ResumableSpaceTimeDijkstra(
    Env *env, int start, int time_horizon, const ReservationTable *rt
) : env(env), start_(start), time_horizon_(time_horizon) {
    env_size_ = env->size();
    rt_ = rt;

    nodes_.emplace(start, Node(nullptr, start, 0, 0));
    openset_.push({0, &nodes_.at(start)});
}

void ResumableSpaceTimeDijkstra::clear() {
    openset_ = Queue();
    nodes_.clear();
}

void ResumableSpaceTimeDijkstra::set_start_node(int start) {
    if (start_ != start) {
        start_ = start;
        clear();

        nodes_.emplace(start, Node(nullptr, start, 0, 0));
        openset_.push({0, &nodes_.at(start)});
    }
}

double ResumableSpaceTimeDijkstra::distance(int node_id) {
    return distance(node_id, -1);
}

double ResumableSpaceTimeDijkstra::distance(int node_id, int time) {
    Node* node = nullptr;

    if (time < 0) {
        double min_distance = -1;
        for (int t = 0; t <= time_horizon_; t++) {
            int st = node_id + t * env_size_;
            if (nodes_.count(st) && nodes_[st].closed) {
                if (min_distance == -1 || nodes_[st].distance < min_distance) {
                    min_distance = nodes_[st].distance;
                    node = &nodes_[st];
                }
            }
        }
    }
    else {
        int st = node_id + time * env_size_;
        if (nodes_.count(st))
            node = &nodes_[st];
    }

    if (!node || !node->closed)
        node = search(node_id, time);

    if (node)
        return node->distance;

    return -1;  // inf
}


Path ResumableSpaceTimeDijkstra::find_path(int node_id) {
    return find_path(node_id, -1);
}

Path ResumableSpaceTimeDijkstra::find_path(int node_id, int time) {
    Node* node = nullptr;

    if (time < 0) {
        double min_distance = -1;
        for (int t = 0; t <= time_horizon_; t++) {
            int st = node_id + t * env_size_;
            if (nodes_.count(st) && nodes_[st].closed) {
                if (min_distance == -1 || nodes_[st].distance < min_distance) {
                    min_distance = nodes_[st].distance;
                    node = &nodes_[st];
                }
            }
        }
    }
    else {
        int st = node_id + time * env_size_;
        if (nodes_.count(st) && nodes_[st].closed)
            node = &nodes_[st];
    }

    if (!node)
        node = search(node_id, time);

    if (node)
        return reconstruct_path(node);

    return {};
}

ResumableSpaceTimeDijkstra::Node* ResumableSpaceTimeDijkstra::search(int goal, int time) {

    auto process_node = [&] (int node_id, double cost, Node* current) {
        int next_time = current->time + 1;
        double distance = current->distance + cost;

        int st = node_id + next_time * env_size_;
        if (!nodes_.count(st)) {
            nodes_.emplace(st, Node(current, node_id, next_time, distance));
            openset_.push({distance, &nodes_.at(st)});
        }
        else if (nodes_.at(st).distance > distance) {
            Node& n = nodes_.at(st);
            n.distance = distance;
            n.parent = current;
            openset_.push({distance, &n});
        }
    };

    while (!openset_.empty()) {
        auto [distance, current] = openset_.top();
        openset_.pop();

        int t = current->time;
        int n = current->node_id;

        if (current->closed)
            continue;

        // expand node
        if (t < time_horizon_) {
            if (!rt_) {
                for (auto &[node_id, cost] : env->get_neighbors(n, false, true))
                    process_node(node_id, cost, current);
            }
            else {
                auto reserved_edges = rt_->get_reserved_edges(t, n);
                for (auto &[node_id, cost] : env->get_neighbors(n, false, true)) {
                    if (!reserved_edges.count(node_id) && !rt_->is_reserved(t + 1, node_id))
                        process_node(node_id, cost, current);
                }
            }
        }

        current->closed = true;

        if (n == goal)
            if (time < 0 || t == time) {
                return current;
            }
        }

    return nullptr;
}

void ResumableSpaceTimeDijkstra::print_node(Node& node) {
    cout << "Node(" << env->node_to_string(node.node_id);
    cout << ", time=" << node.time << ", distance=" << node.distance;
    cout << ", closed=" << node.closed << ")" << endl;
}

Path ResumableSpaceTimeDijkstra::reconstruct_path(Node* node) {
    Path path = {node->node_id};
    while (node->parent != nullptr) {
        node = node->parent;
        path.push_back(node->node_id);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
