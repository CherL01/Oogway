#include <iostream>
#include <math.h>
#include <chrono>
#include <bits/stdc++.h>
using namespace std;
#define V 6

// cost function: Euclidean Distance

double euclidean_dist(std::vector<float> object1, std::vector<float> object2) {
    double x = object1[0] - object2[0];
    double y = object1[1] - object2[1];
    double dist;
    dist = pow(x, 2) + pow(y, 2);
    dist = sqrt(dist);
    return dist;
}

// path solver: brute force

int travllingSalesmanProblem(double graph[V][V], int s) {

    std::vector<vector> possible_solution_list; 

    std::vector<double> vertex;
    for (int i = 0; i < V; i++)
        if (i != s)
            vertex.push_back(i);
 
    int min_path = INT_MAX;
    do {
        int current_pathweight = 0;
        possible_solution_list.erase(possible_solution_list.begin(), possible_solution_list.end());
 
        int k = s;
        for (int i = 0; i < vertex.size(); i++) {
            current_pathweight += graph[k][vertex[i]];
            k = vertex[i];
            possible_solution_list.push_back(k);
        }
        current_pathweight += graph[k][s];

        if (current_pathweight < min_path){
            solution_list.erase(solution_list.begin(), solution_list.end());
            for (int i = 0; i < possible_solution_list.size(); i++) {
                solution_list.push_back(possible_solution_list[i]);
            	}
	    }
 
    } while (
        next_permutation(vertex.begin(), vertex.end()));

    return min_path;
}

// sort euclidean distances into graph

double sort_graph(std::vector<vector<float>> objects) {
    unsigned int objects_size = objects.size();
    double graph[V][V];

    for (unsigned int row = 0; row < objects_size; row++) {
        for (unsigned int col = 0; col < objects_size; col++) {
            graph[row][col] = euclidean_dist(objects[row], objects[col]);
        }
    }

    return graph;

}

int main()
{
    std::vector objects[6];
    // matrix representation of graph
    int graph[][V] = { { 0, 10, 15, 20 },
                       { 10, 0, 35, 25 },
                       { 15, 35, 0, 30 },
                       { 20, 25, 30, 0 } };
    int s = 0;
    cout << travllingSalesmanProblem(graph, s) << endl;
    return 0;
}