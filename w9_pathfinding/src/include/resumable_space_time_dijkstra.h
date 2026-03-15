#pragma once

#include "env.h"
#include "reservation_table.h"


class ResumableSpaceTimeDijkstra {
    // Space-time version of the Resumable Dijkstra algorithm.

    public:
        Env* env;
        
        // terminal_time defines the planning horizon.
        // The search only considers states (node, t) where t <= terminal_time.
        // Paths arriving after terminal_time are not explored.
        ResumableSpaceTimeDijkstra(
            Env* env,
            int start,
            int time_horizon,
            const ReservationTable *rt
        );

        // Returns the current start node of the search.
        int start_node() {return start_;};

        // Changes the start node and resets the search state.
        void set_start_node(int start);

        // Returns the minimal cost from (start, 0) to (node_id, t)
        // for any t <= terminal_time. If unreachable, returns -1.
        double distance(int node_id);

        // Returns the minimal cost from (start, 0) to (node_id, time).
        // If unreachable, returns -1.
        double distance(int node_id, int time);

        // Returns the optimal path from (start, 0) to (node_id, t)
        // for any t <= terminal_time. If unreachable, returns an empty vector.
        Path find_path(int node_id);

        // Returns the optimal path from (start, 0) to (node_id, time).
        // If unreachable, returns an empty vector.
        Path find_path(int node_id, int time);

    private:

        struct Node {
            Node* parent;
            int node_id;
            int time;
            double distance;
            bool closed;

            Node() : parent(nullptr), node_id(-1), time(0), distance(0), closed(false) {}
            Node(Node* parent, int node_id, int time, double distance) :
                parent(parent), node_id(node_id), time(time), distance(distance), closed(false) {}
        };

        int start_;
        int env_size_ = 0;
        int time_horizon_ = 0;
        const ReservationTable* rt_;

        typedef pair<double, Node*> key;
        struct KeyCompare {
            bool operator()(const key& a, const key& b) const {
                if (a.first != b.first)
                    return a.first > b.first;  // smaller cost first

                if (a.second->node_id != b.second->node_id)
                    return a.second->node_id > b.second->node_id;

                return a.second->time > b.second->time; // earlier time first
            }
        };
        typedef priority_queue<key, vector<key>, KeyCompare> Queue;

        Queue openset_;
        std::unordered_map<int, Node> nodes_;

        void clear();
        Node* search(int node_id, int time);
        Path reconstruct_path(Node* node);
        void print_node(Node& node);
};
