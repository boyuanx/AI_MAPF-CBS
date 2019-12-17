#include "CBS.h"
#include <iostream>
#include <queue>
#include <map>
#include <algorithm>

vector<Path> CBS::find_solution() {
    priority_queue<CBSNode*, vector<CBSNode*>, CompareCBSNode> open; // open list

    /* generate the root CBS node */
    auto root = new CBSNode();
    all_nodes.push_back(root);  // whenever generating a new node, we need to
                                 // put it into all_nodes
                                 // so that we can release the memory properly later in ~CBS()

    // find paths for the root node
    root->paths.resize(a_star.ins.num_of_agents);
    for (int i = 0; i < a_star.ins.num_of_agents; i++) {
        root->paths[i] = a_star.find_path(i, list<Constraint>());   // Find paths with no constraints
        if (root->paths[i].empty()) {
            cout << "Fail to find a path for agent " << i << endl;
            return vector<Path>(); // return "No solution"
        }
    }
    // compute the cost of the root node
    for (const auto& path : root->paths)
        root->cost += (int)path.size() - 1;

    // put the root node into open list
    open.push(root);
    while (!open.empty()) {
        auto pNode = open.top();
        open.pop();
        list<Constraint> constraints = find_collision(pNode->paths);
//        for (Constraint constraint: constraints) {
//            int targetTimestep;
//            if (pNode->paths[get<0>(constraint)].size() - 1 < get<3>(constraint)) {
//                targetTimestep = pNode->paths[get<0>(constraint)].size() - 1;
//            } else {
//                targetTimestep = get<3>(constraint);
//            }
//            if (pNode->paths[get<0>(constraint)][targetTimestep] != get<1>(constraint) && get<2>(constraint) == -1) {
//                cout << "Vertex Bug!!" << endl;
//            }
//            if (get<2>(constraint) != -1 && (pNode->paths[get<0>(constraint)][targetTimestep - 1] != get<1>(constraint) || pNode->paths[get<0>(constraint)][targetTimestep] != get<2>(constraint))) {
//                cout << "Edge Bug!!" << endl;
//            }
//        }
        if (constraints.empty()) return pNode->paths;
        for (Constraint constraint : constraints) {
            auto qNode = new CBSNode();
            list<Constraint> temp = pNode->constraints;
            temp.push_back(constraint);
            qNode->constraints = temp;
            qNode->paths = pNode->paths;
            int agent_in_constraint = get<0>(constraint);
            Path path = a_star.find_path(agent_in_constraint, qNode->constraints);
            if (!path.empty()) {
                qNode->paths[agent_in_constraint] = path;
//                for (Constraint constraint: qNode->constraints) {
//                    int targetTimestep;
//                    if (qNode->paths[get<0>(constraint)].size() - 1 < get<3>(constraint)) {
//                        targetTimestep = qNode->paths[get<0>(constraint)].size() - 1;
//                    } else {
//                        targetTimestep = get<3>(constraint);
//                    }
//                    if (qNode->paths[get<0>(constraint)][targetTimestep] == get<1>(constraint) && get<2>(constraint) == -1) {
//                        cout << "Vertex Bug!!" << endl;
//                    }
//                    if (targetTimestep > 0 && qNode->paths[get<0>(constraint)][targetTimestep - 1] == get<1>(constraint) && qNode->paths[get<0>(constraint)][targetTimestep] == get<2>(constraint)) {
//                        cout << "Edge Bug!!" << endl;
//                    }
//                }
                qNode->cost = get_sum_of_cost(qNode->paths);
                open.push(qNode);
            }
            all_nodes.push_back(qNode);
        }
    }
    return vector<Path>(); // return "No solution"
}

// Only looks for the first collision
list<Constraint> CBS::find_collision(const vector<Path>& paths) const {
    list<Constraint> constraints;
    Path longestPath;
    int agent_id = -1;
    for (const Path& path : paths) {
        agent_id++;
        if (path.size() > longestPath.size()) {
            longestPath = path;
        }
    }
    map<int, int> locationsAtPreviousTimestep; // <location, agent_id>
    for (int timestep = 0; timestep < longestPath.size(); timestep++) {
        map<int, int> locationsAtCurrentTimestep; // <location, agent_id>
        agent_id = -1;
        for (const Path& path : paths) {
            agent_id++;
            int currentLocationForCurrentAgent;
            if (timestep > path.size() - 1) {
                currentLocationForCurrentAgent = path[path.size() - 1];
            } else {
                currentLocationForCurrentAgent = path[timestep];
            }
            // Checking for vertex collisions
            auto existingEntry = locationsAtCurrentTimestep.find(currentLocationForCurrentAgent);
            if (existingEntry == locationsAtCurrentTimestep.end()) {
                // No collision
                locationsAtCurrentTimestep.insert(make_pair(currentLocationForCurrentAgent, agent_id));
            } else {
                // Collision
                int conflictingAgentID = existingEntry->second;
                constraints.push_back(make_tuple(agent_id, currentLocationForCurrentAgent, -1, timestep));
                constraints.push_back(make_tuple(conflictingAgentID, currentLocationForCurrentAgent, -1, timestep));
                return constraints;
            }
            // Checking for edge collisions (undetectable by vertex collisions)
            auto existingEntryAtPreviousTimestep = locationsAtPreviousTimestep.find(currentLocationForCurrentAgent);
            if (existingEntryAtPreviousTimestep != locationsAtPreviousTimestep.end() && timestep > 0) {
                int previousLocationForCurrentAgent = path[timestep - 1];
                int potentialConflictingAgent = existingEntryAtPreviousTimestep->second;
                if (potentialConflictingAgent != agent_id && paths[potentialConflictingAgent][timestep-1] == paths[agent_id][timestep] && paths[agent_id][timestep-1] == paths[potentialConflictingAgent][timestep]) {
                    // Collision
                    constraints.push_back(make_tuple(agent_id, previousLocationForCurrentAgent, currentLocationForCurrentAgent, timestep));
                    constraints.push_back(make_tuple(potentialConflictingAgent, currentLocationForCurrentAgent, previousLocationForCurrentAgent, timestep));
                    return constraints;
                }
            }
        }
        locationsAtPreviousTimestep = locationsAtCurrentTimestep;
    }
    return constraints;
}

int CBS::get_sum_of_cost(const vector<Path>& paths) const {
    int sum = 0;
    for (const Path& path : paths) {
        sum += (int)path.size() - 1;
    }
    return sum;
}

CBS::~CBS() {
    // release the memory
    for (auto n : all_nodes)
        delete n;
}