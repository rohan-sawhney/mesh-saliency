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

double Vertex::computeWeightedCurvature(VertexIter root, const double distance2)
{
    std::queue<VertexIter> queue;
    queue.push(root);
    
    std::unordered_map<int, bool> visitedNeighbors;
    visitedNeighbors[index] = true;
    
    // initialize weighted gaussian curvatures
    double weightedCurvature = 0.0;
    double sumExponents = 0.0;
    
    // traverse n ring neighbors
    while (!queue.empty()) {
        VertexIter v = queue.front();
        queue.pop();
        
        HalfEdgeIter h = v->he;
        do {
            int vIndex = h->flip->vertex->index;
            if (!visitedNeighbors[vIndex]) {
                
                VertexIter nv = h->flip->vertex;
                
                if (sqDistances[vIndex] == 0.0) { // cache vertex distances
                    sqDistances[vIndex] = (nv->position - position).squaredNorm();
                    nv->sqDistances[index] = sqDistances[vIndex];
                } 
                
                if (sqDistances[vIndex] < 4 * distance2) {
                    double exponent = exp(-sqDistances[vIndex] / (2 * distance2));
                    weightedCurvature += nv->meanCurvature * exponent;
                    sumExponents += exponent;
                    queue.push(nv);
                }
                
                visitedNeighbors[vIndex] = true;
            }
            
            h = h->flip->next;
            
        } while (h != v->he);
    }

    if (sumExponents > 0.0) return weightedCurvature / sumExponents;
    return 0.0;
}

bool Vertex::isPeakSaliency(VertexIter root, const std::vector<double>& levelSaliencies) const
{
    std::queue<VertexIter> queue;
    queue.push(root);
    
    std::unordered_map<int, bool> visitedNeighbors;
    visitedNeighbors[index] = true;
    
    // traverse 2 ring neighbors
    bool traversed1Ring = false;
    while (!queue.empty()) {
        VertexIter v = queue.front();
        queue.pop();
    
        HalfEdgeIter h = v->he;
        do {
            int vIndex = h->flip->vertex->index;
            if (!visitedNeighbors[vIndex]) {
                
                if (levelSaliencies.empty()) {
                    if (saliency < h->flip->vertex->saliency) return false;
            
                } else {
                    if (levelSaliencies[index] < levelSaliencies[h->flip->vertex->index]) return false;
                }
    
                if (!traversed1Ring) queue.push(h->flip->vertex);
                visitedNeighbors[vIndex] = true;
            }
            
            h = h->flip->next;
        
        } while (h != v->he);
        
        traversed1Ring = true;
    }
    
    return true;
}

