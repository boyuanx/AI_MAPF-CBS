#include "AStarPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm> // reverse
#include <iostream>

ostream& operator<<(ostream& os, const Path& path)
{
    for (auto loc : path) {
        os << loc << " ";
    }
    return os;
}

Path AStarPlanner::make_path(const AStarNode* goal_node) const {
    Path path;
    const AStarNode* curr = goal_node;
    while (curr != nullptr) {
        path.push_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
    return path;
}

Path AStarPlanner::find_path(int agent_id, const list<Constraint>& constraints) {
    int start_location = ins.start_locations[agent_id];
    int goal_location = ins.goal_locations[agent_id];

    // Open list
    priority_queue<AStarNode*, vector<AStarNode*>, CompareAStarNode> open;

    // Unordered map is an associative container that contains key-value pairs with unique keys.
    // The following unordered map is used for duplicate detection, where the key is the location of the node.
    unordered_map<pair<int, int>, AStarNode*, hash_pair> all_nodes;

    int h = ins.get_Manhattan_distance(start_location, goal_location); // h value for the root node
    auto root = new AStarNode(start_location, 0, h, nullptr, 0);
    open.push(root);

    Path path;

    int largestConstraintTimestep = getLargestConstraintTimestep(agent_id, goal_location, constraints);
    while (!open.empty()) {
        auto curr = open.top();
        open.pop();

        // goal test
        if (curr->location == goal_location && curr->timestep > largestConstraintTimestep) {
            path = make_path(curr);
            break;
        }

        // generate child nodes
        for (auto next_location : ins.get_adjacent_locations(curr->location)) {

            if (curr->timestep > ins.map_size()) return path;

            int childTimestep = curr->timestep + 1;
            pair<int, int> nextKey;
            nextKey.first = next_location;
            nextKey.second = childTimestep;
            auto it = all_nodes.find(nextKey);

            if (it == all_nodes.end()) {    // the location has not been visited before
                int next_g = curr->g + 1;
                int next_h = ins.get_Manhattan_distance(next_location, goal_location);
                auto next = new AStarNode(next_location, next_g, next_h, curr, childTimestep);
                if (!satisfiesConstraint(agent_id, next, constraints)) continue;
                open.push(next);
                all_nodes[nextKey] = next;
            }
            // Note that if the location has been visited before,
            // next_g + next_h must be greater than or equal to the f value of the existing node,
            // because we are searching on a 4-neighbor grid with uniform-cost edges.
            // So we don't need to update the existing node.
        }
    }

    // release memory
    for (auto n : all_nodes)
        delete n.second;

    return path;
}

bool AStarPlanner::satisfiesConstraint(int agent_id, const AStarNode *node, const list<Constraint> &constraints) const {
    for (auto constraint : constraints) {
        if (get<0>(constraint) != agent_id) continue;
        if (get<2>(constraint) == -1) {
            // Vertex constraint
            if (get<1>(constraint) == node->location && get<3>(constraint) == node->timestep) return false;
            if (get<3>(constraint) == -1 && get<1>(constraint) == node->location) return false;
            continue;
        }
        // Edge constraint
        if (get<1>(constraint) == node->parent->location && get<2>(constraint) == node->location && get<3>(constraint) == node->timestep) return false;
    }
    return true;
}

int AStarPlanner::getLargestConstraintTimestep(int agent_id, int goal_location, const list<Constraint>& constraints) const {
    int target = 0;
    for (auto constraint : constraints) {
        if (get<0>(constraint) != agent_id) continue;
        if (get<1>(constraint) != goal_location) continue;
//        if (get<2>(constraint) != -1) continue;
        if (get<3>(constraint) > target) target = get<3>(constraint);
    }
    // cout << "Target: " << target << endl;
    return target;
}