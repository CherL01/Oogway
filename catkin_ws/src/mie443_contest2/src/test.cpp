#include "path_planning.cpp"
using namespace std;

int main()
{
    vector<float> v0 = {0, 7, 25};
    vector<float> v1 = {0, 0, 25};
    vector<float> v2 = {0, 5, 50};
    vector<float> v3 = {3, 5, 25};
    vector<float> v4 = {3, 0, 50};

    vector<vector<float>> test_graph = {v0, v2, v1, v4, v3};
    vector<vector<float>> sorted_graph = sortGraph(test_graph);
    int start_node = 0;

    vector<int> min_path = travllingSalesmanProblem(sorted_graph, start_node);

    for (auto i: min_path) 
        cout << i << ' ';
}