#include "Mesh.h"
#include "MeshIO.h"
#include "BoundingBox.h"

Mesh::Mesh()
{
    
}

Mesh::Mesh(const Mesh& mesh)
{
    *this = mesh;
}

bool Mesh::read(const std::string& fileName)
{
    std::ifstream in(fileName.c_str());

    if (!in.is_open()) {
        std::cerr << "Error: Could not open file for reading" << std::endl;
        return false;
    }
    
    bool readSuccessful = false;
    if ((readSuccessful = MeshIO::read(in, *this))) {
        normalize();
    }
    
    return readSuccessful;
}

bool Mesh::write(const std::string& fileName) const
{
    std::ofstream out(fileName.c_str());
    
    if (!out.is_open()) {
        std::cerr << "Error: Could not open file for writing" << std::endl;
        return false;
    }
    
    MeshIO::write(out, *this);
    
    return false;
}

void Mesh::buildLaplacian(Eigen::SparseMatrix<double>& L) const
{
    std::vector<Eigen::Triplet<double>> LTriplet;
    
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        
        HalfEdgeCIter he = v->he;
        double dualArea = v->dualArea();
        double sumCoefficients = 0.0;
        do {
            // (cotA + cotB) / 2A
            double coefficient = 0.5 * (he->cotan() + he->flip->cotan()) / dualArea;
            sumCoefficients += coefficient;
            
            LTriplet.push_back(Eigen::Triplet<double>(v->index, he->flip->vertex->index, coefficient));
            
            he = he->flip->next;
        } while (he != v->he);
        
        LTriplet.push_back(Eigen::Triplet<double>(v->index, v->index, -sumCoefficients));
    }
    
    L.setFromTriplets(LTriplet.begin(), LTriplet.end());
}

void Mesh::computeMeanCurvature()
{
    int v = (int)vertices.size();
    Eigen::SparseMatrix<double> L(v, v);
    buildLaplacian(L);
    
    Eigen::MatrixXd x;
    x.resize(v, 3);
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        x.row(v->index) = v->position;
    }
    x = L * x;
    
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->meanCurvature = 0.5 * x.row(v->index).norm();
    }
}

void Mesh::computeSaliency(const int levels)
{
    int count = (int)vertices.size();
    double minSaliency = INFINITY;
    double maxSaliency = -INFINITY;
    std::stack<VertexIter> stack;
    std::vector<double> levelSaliencies(count);
    
    // 1: compute mean curvature
    computeMeanCurvature();
    
    // 2: initialize and compute extent
    BoundingBox bbox;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        bbox.expandToInclude(v->position);
    }
    double extent = 0.003 * 0.003 * bbox.extent.squaredNorm();
    
    // 3
    for (int i = 0; i < levels; i++) {
        double sumSaliency = 0.0;
        
        // compute level saliencies
        double distance2 = (i+2)*(i+2)*extent;
        for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
            
            stack.push(v); double weightedCurvature1 = v->computeWeightedCurvature(stack, distance2);
            stack.push(v); double weightedCurvature2 = v->computeWeightedCurvature(stack, 4*distance2);
            
            levelSaliencies[v->index] = std::abs(weightedCurvature1 - weightedCurvature2);
            sumSaliency += levelSaliencies[v->index];
            std::cout << "v: " << v->index << std::endl;
        }
        
        // normalize
        double maxLevelSaliency = -INFINITY;
        for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
            levelSaliencies[v->index] /= sumSaliency;
            if (maxLevelSaliency < levelSaliencies[v->index]) maxLevelSaliency = levelSaliencies[v->index];
        }
        
        // compute mean of local maxima
        double peaks = 0.0;
        double meanLocalMaxSaliency = 0.0;
        for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
            if (v->isPeakSaliency(levelSaliencies) && levelSaliencies[v->index] != maxLevelSaliency) {
                meanLocalMaxSaliency += levelSaliencies[v->index];
                peaks += 1.0;
            }
        }
        meanLocalMaxSaliency /= peaks;
        
        // apply non-linear suppression operator to level saliency
        double suppressionFactor = pow(maxLevelSaliency - meanLocalMaxSaliency, 2);
        for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
            v->saliency += levelSaliencies[v->index] * suppressionFactor;
            
            if (i+1 == levels) {
                if (v->saliency < minSaliency) minSaliency = v->saliency;
                if (maxSaliency < v->saliency) maxSaliency = v->saliency;
            }
        }
    }
    
    // 4: scale between 0 and 1
    double dSaliency = maxSaliency - minSaliency;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->saliency = (v->saliency - minSaliency) / dSaliency;
    }
}

void Mesh::normalize()
{
    // compute center of mass
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        cm += v->position;
    }
    cm /= (double)vertices.size();
    
    // translate to origin
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position -= cm;
    }
    
    // determine radius
    double rMax = 0;
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        rMax = std::max(rMax, v->position.norm());
    }
    
    // rescale to unit sphere
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position /= rMax;
    }
}
