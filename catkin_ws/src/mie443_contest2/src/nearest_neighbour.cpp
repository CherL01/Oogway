#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <set>
#include <cassert>

struct object {
    double x, y, phi; // coordinates of object
    int cluster; // cluster that object belongs to

    object():
        x(0.0),
        y(0.0),
        cluster(-1) {}

    double distance(object ob) {
        return std::hypot(ob.x - x, ob.y - y);
    }
};

void kmeans(vector<object>* objects, int epochs, int k);

vector<tuple<double, double, double>> objects = {{1.0, 2.0, 0.0}, {1.0, 3.0, 0.0}, {5.0, 7.0, 0.0}, {2.0, 1.0, 0.0}, {8.0, 10.0, 0.0}};

kmeans(&objects);

vector<object> centroids;
srand(time(0));
for (int i = 0; i < k; ++i) {
    centroids.push_back(objects->at(rand() % n));
}

for (vector<object>::iterator c = begin(centroids));
    c != end(centroids); ++c {}

