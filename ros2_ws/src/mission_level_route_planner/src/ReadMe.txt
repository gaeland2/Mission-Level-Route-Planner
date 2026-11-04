How to use:
Only edit values in main
Edit numbers in vector<vector<pair<double,double>>> a to change values:
    Add an addtional vector into the main vector = adds a new node
    Add an additional pair into a smaller vector = adds new edge from smaller vector
    first value in pair = node that edge connects to (that is not smaller vector number)
    second value in pair = edge weight
Edit fucntion vector<double> b = dijkstra(a,0,4), first number is start and final number is destination

Once done and saved, complie using
    g++ dijkstra.cpp

then run exe file for output