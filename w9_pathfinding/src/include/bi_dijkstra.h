 #pragma once

#include "pf.h"


class BiDijkstra : public AbsPathFinder {
    // Bidirectional Dijkstra

    typedef pair<double, int> key;  // first - distance, second - node_id
    typedef priority_queue<key, vector<key>, std::greater<key>> Queue; 

    struct Node {
        int parent;
        double distance;  // from start to this node

        Node() : parent(-1), distance(-1) {};

        void clear() {
            parent = -1;
            distance = -1;
        }
    };

    public:
        Env* env;
        BiDijkstra(Env* env);

        vector<int> find_path(int start, int end);

    private:
        double epsilon = 0.000001;
        vector<Node> forward_nodes_, backward_nodes_;
        vector<int> workset_;
        vector<bool> closedset_;
        vector<int> reconstruct_path(int start, int end); 
        void clear();
        bool step(int side, Queue &queue, vector<Node> &nodes);
};
