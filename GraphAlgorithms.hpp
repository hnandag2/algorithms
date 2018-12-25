//
//  GraphAlgorithms.hpp
//
//  C++ implementation of popular graph algorithms (eg: min spanning tree, shortest path, etc.)
//
//  Created by Nandagopal, Harihar on 12/24/18.
//

# pragma once 

#include <stdio.h>

#include <list>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <iostream>
#include "UnionFind.h"

template<typename T>
class Graph
{
public:
    // bidir represents undirected/directed, undirected by default
    void addEdge(T u, T v, int dist, bool bidir = true);
    
    // print adjacency list of the graph (member variable)
    void printAdjacencyList();
    
    // print any adjacency list
    static void printAdjacencyList(std::unordered_map<T, std::list<std::pair<T, int>>> adjacencyList);
    
    // return: adjacency list of the min spanning tree
    std::unordered_map<T, std::list<std::pair<T,int>>> kruskalsMinimumSpanningTree();
    
private:
    std::unordered_map<T, std::list<std::pair<T, int>>> mAdjacencyList;
};