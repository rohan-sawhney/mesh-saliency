#ifndef VERTEX_H
#define VERTEX_H

#include "Types.h"

class Vertex {
public:
    // outgoing halfedge
    HalfEdgeIter he;
    
    // location in 3d
    Eigen::Vector3d position;
    
    // id between 0 and |V|-1
    int index;
    
    // mean curvature
    double meanCurvature;
    
    // saliency
    double saliency;
    
    // vertex distances
    std::unordered_map<int, double> sqDistances;
    
    // checks if vertex is contained in any edge or face
    bool isIsolated() const;
    
    // returns area of barycentric dual cell associated with the vertex
    double dualArea() const;
    
    // computes saliency given cut-off distance
    double computeWeightedCurvature(std::stack<VertexIter>& stack, const double distance2);
    
    // checks is saliency is a maximum in local neighborhood
    bool isPeakSaliency(const std::vector<double>& levelSaliencies = std::vector<double>()) const;
};

#endif
