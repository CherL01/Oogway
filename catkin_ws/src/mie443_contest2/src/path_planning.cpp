#include <iostream>
#include <math.h>
#include <chrono>
#include <bits/stdc++.h>
using namespace std;

// cost function: Euclidean Distance

float euclideanDist(vector<float> object1, vector<float> object2) {
    float x = object1[0] - object2[0];
    float y = object1[1] - object2[1];
    float dist;
    dist = pow(x, 2) + pow(y, 2);
    dist = sqrt(dist);
    return dist;
}

// path solver: brute force

vector<int> travellingSalesmanProblem(vector<vector<float>> graph, int s) {

    // put all nodes other than start into a vector
    vector<int> vertex;
    for (int i = 0; i < graph.size(); i++)
        if (i != s)
            vertex.push_back(i);
 
    // save minimum path
    float min_path_cost = INT_MAX;
    vector<int> min_path = vertex;
    do {
        float current_pathweight = 0;
 
        int k = s;
        for (int i = 0; i < vertex.size(); i++) {
            current_pathweight += graph[k][vertex[i]];
            k = vertex[i];
        }
        current_pathweight += graph[k][s];

        if (current_pathweight < min_path_cost){
            min_path.clear();
            for (auto i: vertex) 
                min_path.push_back(i);
	    }

        // update minimum
        min_path_cost = min(min_path_cost, current_pathweight);

        // for (auto i: vertex) 
        //     cout << i << ' ';
        // cout << "\n";
        // cout << current_pathweight;
        // cout << "\n";
 
    } while (
        next_permutation(vertex.begin(), vertex.end()));

    return min_path;
}

// sort euclidean distances into graph

vector<vector<float>> sortGraph(vector<vector<float>> objects) {
    int objects_size = objects.size();
    vector<vector<float>> graph;

    for (int row = 0; row < objects_size; row++) {
        vector<float> temp_row;
        for (int col = 0; col < objects_size; col++) {
            temp_row.push_back(euclideanDist(objects[row], objects[col]));
        }
        graph.push_back(temp_row);
    }

    return graph;

}

// int main()
// {
//     vector objects[6];
//     // matrix representation of graph
//     int graph[][V] = { { 0, 10, 15, 20 },
//                        { 10, 0, 35, 25 },
//                        { 15, 35, 0, 30 },
//                        { 20, 25, 30, 0 } };
//     int s = 0;
//     cout << travllingSalesmanProblem(graph, s) << endl;
//     return 0;
// }