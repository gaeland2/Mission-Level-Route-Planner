#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <utility>
#include <functional>
#include <climits>
using namespace std;
#ifndef Dijkstras
#define Dijkstras


vector<double> dijkstra(vector<vector<pair<double,double>>>& adj, double src, double dest, function<double(int)> heuristic = [](int node) {return 0.0;});


#endif
