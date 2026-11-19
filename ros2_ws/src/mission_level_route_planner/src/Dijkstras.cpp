#include "Dijkstras.h"
#include <functional>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <utility>
#include <functional>
#include <climits>

//Yes I am not putting a namespace on here, OctoMap is 100% not using Dijkstra, else I would not need to WRITE THIS SHIT

// Ok updates I think I could implement:
//Mabye I can make the positioning script in here? <abnye make it a point3d and redo it over to check points and give them scores?
// I also don't think I "need" dijkstra's to be super duper specific?

vector<double> dijkstra(vector<vector<pair<double,double>>>& adj, double src, double dest,function<double(int)> heuristic) {
/* This will compare nodes to find the shortest path from a start point (src) and a end point (dest). Importantly, it will be a vector in a vector in a pair for each nodes and the point value to and from. 
Updates will include: reducing node limit by using the order of the nodes to calcuate, and making the first node the start and last node the end (currently not added because I don't have the entire information) */
    double V = adj.size();

    
    priority_queue<pair<double, double>, vector<pair<double, double>>, greater<pair<double, double>>> pq;

    vector<double> dist(V, INT_MAX);
    vector<double> parent(V,-1);

    
    dist[src] = 0;
    pq.emplace(0, src);

    
    while (!pq.empty()) {
        auto top = pq.top();
        pq.pop();

        double d = top.first;  
        double u = top.second; 

        
        if (d > dist[u]+ heuristic(u))
            continue;

        
        for (auto &p : adj[u]) {
            double v = p.first; 
            double w = p.second; 

            
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;  
                parent[v] = u; 
                pq.emplace(dist[v]+heuristic(v), v);
            }
        }
    }
    vector<double> path;

        for (double v = dest; v != -1; v = parent[v]) {
        path.push_back(v);
        }
    reverse(path.begin(),path.end());
    return path;

}

int main() { //testing value to make sure it prints correctly, delete when it is pushed
    vector<vector<pair<double,double>>> a(6);
    a[0] ={{1,3},{2,99},{3,99},{4,5},{5,99}};
    a[1] ={{0,3},{2,2.5},{3,99},{4,99},{5,5.5}};
    a[2] ={{0,99},{1,2.5},{3,2},{4,99},{5,2.5}};
    a[3] ={{0,99},{1,99},{2,2},{4,1},{5,3.5}};
    a[4] ={{0,5},{1,99},{2,99},{3,1},{5,99}};
    a[5] = {{0,99},{1,5.5},{2,2.5},{3,3.5},{4,99}};

    
    int heuristicCalls = 0;

    vector<double> b = dijkstra(a,0,5);
    for (double i = 0; i < b.size(); i++) {
        std::cout << b[i] <<"->";
    }
    std::cout << endl;
    std::cout << "Now for the hertical hook" << endl;
    vector <double> estimates = {5,4,3,2,1,0};
    auto h = [&](int node) -> double {
    heuristicCalls++;
      if (node < 0 || static_cast<size_t>(node) >= estimates.size()) {
        cerr << "Invalid heuristic node: " << node << endl;
        return 0.0;
    }
    return estimates[node];};
    vector<double> c = dijkstra(a,0,5,h);
    for (double i = 0; i < c.size(); i++) {
        std::cout << c[i] <<"->";
    }
    return 0;
}