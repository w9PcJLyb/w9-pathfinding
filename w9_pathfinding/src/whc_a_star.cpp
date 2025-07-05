#include "unordered_map"

#include "include/whc_a_star.h"


WHCAStar::WHCAStar(Env* env) : env(env), st_a_star_(env) {
}

vector<Path> WHCAStar::mapf(vector<int> starts, vector<int> goals) {
    return mapf(starts, goals, 100, 16, nullptr);
}

vector<Path> WHCAStar::mapf(
    vector<int> starts,
    vector<int> goals,
    int max_length,
    int window_size,
    const ReservationTable *rt
) {
    assert(starts.size() == goals.size());

    if (starts.size() == 0)
        return {};

    ReservationTable reservation_table(env->size());
    if (rt)
        reservation_table = *rt;

    vector<Agent> agents;
    agents.reserve(starts.size());
    for (size_t agent_id = 0; agent_id < starts.size(); agent_id++) {
        int start = starts[agent_id];
        int goal = goals[agent_id];
        auto rrs = st_a_star_.reverse_resumable_search(goal);
        agents.emplace_back(start, goal, std::move(rrs));
    }

    return mapf_(agents, max_length, window_size, reservation_table);
}

vector<Path> WHCAStar::mapf_(
    vector<Agent> &agents,
    int max_length,
    int window_size,
    ReservationTable &rt
) {
    bool edge_collision = env->edge_collision();

    bool all_agents_done = false;
    while (!all_agents_done) {
        all_agents_done = true;

        for (Agent& agent : agents) {
            int time = agent.path.size() - 1;
            int min_terminal_time = rt.last_time_reserved(agent.goal);

            if (agent.position() == agent.goal && min_terminal_time <= time) {
                // Agent has reached its goal and can stay without causing conflicts
                continue;
            }

            if (min_terminal_time > max_length) {
                // Goal is reserved beyond the planning horizon — cannot reach it safely
                return {};
            }

            int active_window = std::min(window_size, max_length - time);
            if (active_window <= 0)
                return {};

            Path path = st_a_star_.find_path_with_depth_limit(
                agent.position(),
                agent.goal,
                active_window,
                &rt,
                agent.rrs.get(),
                min_terminal_time,
                time
            );
            if (path.empty())
                return {};

            all_agents_done = false;
            agent.add_path(path);
            rt.add_path(time, path, false, edge_collision);
        }
    }

    vector<Path> paths;
    for (auto &agent: agents)
        paths.push_back(agent.path);

    return paths;
}
