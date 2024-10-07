/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include <iostream>
#include <map.h>
#include <limits>
#include <random.h>
using namespace std;


struct vertex {
    Color color = GRAY;
    double length = 0;
    Loc parent;
    Loc location;
};

/*
This function creates map for all the vertex that exists in the world
map key is vertex Location, and value is vertec istself
*/
Map<Loc, vertex> initialVertecies(Grid<double>& world) {
    Map<Loc, vertex> result;
    for (int i = 0; i < world.nRows; i++) {
        for (int j = 0; j < world.nCols; j++) {
            vertex vert;
            Loc location = makeLoc(i, j);
            vert.location = location;
            result[location] = vert;
        }
    }
    return result;
}

//This function adds all the location from end vertex to start vertex
Vector<Loc> convertLocationToVector(vertex vert, Map<Loc, vertex>& vertecies, Grid<double>& world) {
    Vector<Loc> result;
    while (vert.parent.row != -1) {
        result.insert(0, vert.location);
        vert = vertecies[vert.parent];
    }
    result.insert(0, vert.location);

    return result;
}

// This function returns all the possible neighbors for current vertex
Set<Loc> getAllNeighbors(vertex& curr, Grid<double>& world, Map<Loc, vertex>& vertecies) {
    Set<Loc> result;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            if (i == 0 && j == 0) continue;
            else {
                int nrow = curr.location.row + i;
                int ncol = curr.location.col + j;
                if (world.inBounds(nrow, ncol)) {
                    Loc loc = makeLoc(nrow, ncol);
                    if (vertecies[loc].color != GREEN) {
                        result.add(loc);
                    }
                }
            }
        }
    }
    return result;
}

// This function finds the shorthest path from start to end location, it returns 
//all the Locations of the path
Vector<Loc> 
shortestPath(Loc start, Loc end, Grid<double>& world,
    double costFunction(Loc one, Loc two,
        Grid<double>& world),
    double heuristic(Loc start, Loc end, Grid<double>& world))
{

    Map<Loc, vertex> vertecies = initialVertecies(world);
    TrailblazerPQueue<Loc> queue;
    queue.enqueue(start, heuristic(start,end,world));
    vertecies[start].color = YELLOW;
    Loc startParent = makeLoc(-1, -1);
    vertecies[start].parent = startParent;
    while (!queue.isEmpty()) {
        Loc curr = queue.dequeueMin();
        vertecies[curr].color = GREEN;
        colorCell(world, curr, GREEN);
        if (curr == end) {
            return convertLocationToVector(vertecies[curr], vertecies, world);
        }
        Set<Loc> neighbors = getAllNeighbors(vertecies[curr], world, vertecies);
        for (Loc loc : neighbors) {
            double len = costFunction(vertecies[curr].location, vertecies[loc].location, world);
            if (vertecies[loc].color == GRAY) {
                vertecies[loc].color = YELLOW;
                vertecies[loc].length = vertecies[curr].length + len;
                vertecies[loc].parent = curr;
                queue.enqueue(loc, vertecies[loc].length+ heuristic(loc,end,world));
            }
            else if (vertecies[loc].color == YELLOW) {
                if (vertecies[loc].length > vertecies[curr].length + len) {
                    vertecies[loc].length = vertecies[curr].length + len;
                    vertecies[loc].parent = curr;
                    queue.decreaseKey(loc, vertecies[loc].length+ heuristic(loc, end, world));
                }
            }
        }
    }

    return Vector<Loc>();
}

//This function returns all the possible neighbors for (i,j) lcoation, 
//it does not includes diagonal ones
Set<Loc> getEndLoc(int i, int j, Grid<int>& world) {
    Set<Loc> result;
    for (int ik = -1; ik <= 1; ik++) {
        for (int jk = -1; jk <= 1; jk++) {
            if (abs(ik)-abs(jk)==0) continue;
            else {
                if (world.inBounds(i + ik, j + jk)) {
                    result.add(makeLoc(i + ik, j + jk));
                }
            }
        }
    }
    return result;
}

//This function creates graph for edges, it adds each edges to queue with random weight
TrailblazerPQueue<Edge> initialEdges(int numRows, int numCols) {
    Grid<int> world(numRows, numCols);
    TrailblazerPQueue<Edge> result;
    Set<Edge> used;
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            Loc loc = makeLoc(i, j);
            Set<Loc> endLocs = getEndLoc(i, j,world);
            for (Loc end : endLocs) {
                Edge edge = makeEdge(loc, end);
                Edge revEdge = makeEdge(end, loc);
                if (!used.contains(edge)&& !used.contains(revEdge)) {
                    result.enqueue(edge,randomReal(0,1)*10);
                    used.add(edge);
                    used.add(revEdge);
                }
            }
        }
    }
    return result;
}

//This function makes merges of two clusters, maybe there is not even cluster
//for loc, so it just adds that loc to cluster
void addCluster(Loc &start, Loc &end,Set<Loc>* & startCluster, Set<Loc>* & endCluster,
    Vector< Set<Loc> > &allClusters , Set<Edge> &result) {
    if (startCluster==NULL && endCluster == NULL) {
        Set<Loc> cluster;
        cluster += start;
        cluster += end;
        allClusters.add(cluster);
    }
    else if (startCluster == NULL && endCluster != NULL) {
        endCluster->add(start);
    }
    else if (startCluster != NULL && endCluster == NULL) {
        startCluster->add(end);
    }
    else if (startCluster != NULL && endCluster != NULL) {
        for (Loc loc : *endCluster) {
            startCluster->add(loc);
        }
        for (int i = 0; i < allClusters.size(); i++) {
            if (&allClusters[i] == endCluster) {
                allClusters.remove(i);
                break;
            }
        }
    }
    result.add(makeEdge(start, end));
}

//This function returns cluster(Set of locs) if that cluster contains that location
//else it returns NULL
Set<Loc>* getCluster(Vector< Set<Loc> >&allClusters,Loc loc) {
    for (Set<Loc> &cluster : allClusters) {
        if (cluster.contains(loc)) {
            return &cluster;
        }
    }
    
    return NULL;
}

//This function creates maze, it returns set of edges
Set<Edge> createMaze(int numRows, int numCols) {
    Set<Edge> result;
    TrailblazerPQueue<Edge> edges = initialEdges(numRows, numCols);
    Vector< Set<Loc> > allClusters;
    while (!edges.isEmpty()) {
        Edge curr = edges.dequeueMin();
        Loc start = curr.start;
        Loc end = curr.end;
        Set<Loc>* startCluster = getCluster(allClusters, start);
        Set<Loc>* endCluster = getCluster(allClusters, end);
        if (startCluster!=NULL && endCluster!=NULL && startCluster==endCluster) continue;
        else {
            addCluster(start,end,startCluster, endCluster, allClusters,result);
        }
 
    }

    return result;
}