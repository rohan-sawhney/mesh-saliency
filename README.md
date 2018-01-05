# Mesh Saliency 

Implements Mesh Saliency by Lee et al., a human perception inspired importance measure to process and view 3d meshes. Compared to a local measure of shape such as curvature, the saliency approach is able identity regions that are different from their surrounding context. For example, repeated patterns, even if high in curvature, are regarded as visually monotonous while flat regions in the middle of repeated bumps are recorded as being important. Applications of this perception based metric include saliency guided mesh simplification, viewpoint selection for 3d databases and interest point detection. 

Note: Requires Eigen 3.2.4 and assumes it is in /usr/local/Cellar/eigen/3.2.4/include/eigen3/
