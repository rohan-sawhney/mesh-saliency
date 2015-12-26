#include "Vertex.h"
#include "HalfEdge.h"
#include "Face.h"

std::vector<HalfEdge> isolated;

bool Vertex::isIsolated() const
{
    return he == isolated.begin();
}

double Vertex::dualArea() const
{
    double area = 0.0;
    
    HalfEdgeCIter h = he;
    do {
        area += h->face->area();
        h = h->flip->next;
        
    } while (h != he);
    
    return area / 3.0;
}

double Vertex::computeWeightedCurvature(std::stack<VertexIter>& stack, const double distance2)
{
    std::vector<bool> visitedNeighbors((int)sqDistances.size());
    visitedNeighbors[index] = true;
    
    // initialize weighted gaussian curvatures
    double weightedCurvature = 0.0;
    double sumExponents = 0.0;
    
    // traverse n ring neighbors
    while (!stack.empty()) {
        VertexIter v = stack.top();
        stack.pop();
        
        HalfEdgeIter h = v->he;
        do {
            VertexIter nv = h->flip->vertex;
            
            int vIndex = nv->index;
            if (!visitedNeighbors[vIndex]) {
                
                if (sqDistances[vIndex] == 0.0) { // cache vertex distances
                    sqDistances[vIndex] = (nv->position - position).squaredNorm();
                    if (nv->sqDistances[index] == 0.0) nv->sqDistances[index] = sqDistances[vIndex];
                } 
                
                if (sqDistances[vIndex] < 4 * distance2) {
                    double exponent = exp(-sqDistances[vIndex] / (2 * distance2));
                    weightedCurvature += nv->meanCurvature * exponent;
                    sumExponents += exponent;
                    stack.push(nv);
                }
                
                visitedNeighbors[vIndex] = true;
            }
            
            h = h->flip->next;
            
        } while (h != v->he);
    }

    if (sumExponents > 0.0) return weightedCurvature / sumExponents;
    return 0.0;
}

bool Vertex::isPeakSaliency(const std::vector<double>& levelSaliencies) const
{
    HalfEdgeIter h = he;
    do {
        if (levelSaliencies[index] < levelSaliencies[h->flip->vertex->index]) {
            return false;
        }
        
        h = h->flip->next;
        
    } while (h != he);
    
    return true;
}

