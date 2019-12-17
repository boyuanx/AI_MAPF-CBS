#include <iostream>
#include <fstream>
#include "MAPFInstance.h"
#include "AStarPlanner.h"
#include <tuple>
#include <chrono>

int main(int argc, char *argv[]) {
    MAPFInstance ins;
    string input_file = argv[1];
    string output_file = argv[2];
    if (ins.load_instance(input_file)) {
        ins.print_instance();
    } else {
        cout << "Fail to load the instance " << input_file << endl;
        exit(-1);
    }

    auto begin = chrono::high_resolution_clock::now();

    AStarPlanner a_star(ins);
    vector<Path> paths(ins.num_of_agents);

    // assign priority ordering to agents
    // By default, we use the index ordering of the agents where
    // the first always has the highest priority.
    list<int> priorities;
    for (int i = 0; i < ins.num_of_agents; i++) {
        priorities.push_back(i);
    }

    // plan paths
    list<Constraint> constraints;
    for (int i : priorities) {
        paths[i] = a_star.find_path(i, constraints);

        if (paths[i].empty()) {
            cout << "Fail to find any solutions for agent " << i << endl;
            auto end = chrono::high_resolution_clock::now();
            auto dur = end - begin;
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
            cout << "CPU time (ms): " << ms << endl;
            return 0;
        }

        // Transforms planned paths into constraints
        for (int agent : priorities) {
            if (agent == i) continue;
            for (Path path : paths) {
                int timestep = 0;
                int previousLocation;
                for (int location : path) {
                    if (timestep > 0) constraints.push_back(make_tuple(agent, location, previousLocation, timestep));
                    if (timestep == path.size() - 1) {
                        // timestep == -1 is the goal state flag, always check in AStarPlanner
                        constraints.push_back(make_tuple(agent, location, -1, -1));
                    } else {
                        constraints.push_back(make_tuple(agent, location, -1, timestep));
                    }
                    timestep++;
                    previousLocation = location;
                }
            }
        }
    }

    auto end = chrono::high_resolution_clock::now();
    auto dur = end - begin;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    cout << "CPU time (ms): " << ms << endl;

    // print paths
    cout << "Paths:" << endl;
    int sum = 0;
    for (int i = 0; i < ins.num_of_agents; i++) {
        cout << "a" << i << ": " << paths[i] << endl;
        sum += (int)paths[i].size() - 1;
    }
    cout << "Sum of cost: " << sum << endl;

    // save paths
    ofstream myfile (output_file.c_str(), ios_base::out);
    if (myfile.is_open()) {
        for (int i = 0; i < ins.num_of_agents; i++) {
            myfile << paths[i] << endl;
        }
        myfile.close();
    } else {
        cout << "Fail to save the paths to " << output_file << endl;
        exit(-1);
    }
    return 0;
}