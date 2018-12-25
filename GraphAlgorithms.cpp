//
//  GraphAlgorithms.cpp
//
//  Created by Nandagopal, Harihar on 12/24/18.
//

#include "GraphAlgorithms.hpp"

template<typename T>
void Graph<T>::addEdge(T u, T v, int dist, bool bidir)
{
    mAdjacencyList[u].push_back(std::make_pair(v, dist));
    if (bidir)
        mAdjacencyList[v].push_back(std::make_pair(u, dist));
    else
        mAdjacencyList[v] = std::list<std::pair<T, int>>();
}

template<typename T>
void Graph<T>::printAdjacencyList()
{
    for (auto& vertex : mAdjacencyList)
    {
        // vertex. first is the vertex
        std::cout << vertex.first << "->";
        // print each v,w pair to the vertex
        for (auto& vw : vertex.second)
        {
            std::cout << "(" << vw.first << "," << vw.second << ")";
        }
        std::cout << std::endl;
    }
}

template<typename T>
void Graph<T>::printAdjacencyList(std::unordered_map<T, std::list<std::pair<T, int>>> adjacencyList)
{
    for (auto& vertex : adjacencyList)
    {
        // vertex. first is the vertex
        std::cout << vertex.first << "->";
        // print each v,w pair to the vertex
        for (auto& vw : vertex.second)
        {
            std::cout << "(" << vw.first << "," << vw.second << ")";
        }
        std::cout << std::endl;
    }
}

template<typename T>
std::unordered_map<T, std::list<std::pair<T,int>>> Graph<T>::kruskalsMinimumSpanningTree()
{
    // adjacency list of the minimum spanning tree - return value
    std::unordered_map<T, std::list<std::pair<T, int>>> minSpanTree;
    
    // sorted map of the edges
    std::multimap<int, std::pair<T,T>> edgeMap;
    
    // union find data struct needed for Kruskal's algorithm
    UnionFind<T> uf;
    
    // sort the edges by putting them into a map (O(nlogn))
    // + add vertices to the uf data struct
    // + initialize the minSpanTree with each vertice
    for (auto& vertex : mAdjacencyList)
    {
        uf.insert(vertex.first);
        
        minSpanTree.insert(std::pair<T, std::list<std::pair<T, int>>>(vertex.first, std::list<std::pair<T, int>>()));
        
        auto& l = vertex.second;
        
        for (auto it = l.begin(); it != l.end(); it++)
        {
            auto p = std::pair<T,T>(vertex.first, it->first);
            edgeMap.insert(std::pair<int, std::pair<T,T>>(it->second, p));
        }
    }
    
    // Kruskal's algorithm
    for (auto it = edgeMap.begin(); it != edgeMap.end(); it++)
    {
        auto edgePair = it->second;
        auto v1 = edgePair.first;
        auto v2 = edgePair.second;
        
        // need to unionize and then add this edge to the minSpanTree
        if (uf.find(v1) != uf.find(v2))
        {
            uf.unionize(v1, v2);
            
            // insert edge into minSpanTree
            minSpanTree[v1].push_back(std::pair<T,int>(v2, it->first));
        }
    }
    
    return minSpanTree;
}

// this definition prevents linker errors when instantiating Graph objects in other cpps
template class Graph<int>;
