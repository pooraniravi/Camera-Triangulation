Triangulation is a method used to compute 3D coordinates of a point in space using the projections of that point in atleast 2 images. I tried implementing both linear and midpoint triangulation.

1. Computed the intrinsic matrix (K) given the cam parameters.
2. Found the cam poses at locations 1 and 2 from the given data which I then used to compute the extrinsic matrices E1 and E2. 
   It was a little tricky to get the rotations and coordinate transformations right in the first go. To verify if I got my extrinsics right- 
    a) Converted cam origin to world coordinates using the extrinsic matrix and verified it with given cam centers
    b) Unprojected principal ray in world coordinates
3. Computed cam projection matrices at both locations by multiplying the intrinsic and extrinsic matrices. To verify this, I unprojected the center pixel 
   to see if we get the principal ray of the cam. I also verified if the cam centers are in the null space of the computed matrices.
4. Here comes the main theory- 
    a) For linear triangulation- With the projection matrices and pixel coordinates in homogeneous system, similarity relations can be established. 
       This similarity can be removed by taking cross product as both are in the same direction, giving rise to x x PX=0. i.e AX=0. 
       One 2D to 3D point correspondence gives 2 equations and we have 3 unknowns. So, with 2 given correspondences, the homogeneous linear system can be solved. 
       Computed SVD to solve this linear system and found the eigen vector corresponding to the smallest eigen value. 
       Normalized and converted this to euclidean coordinates. This gives the answer.
    b) For midpoint triangulation- I found the two rays originating from the pixel coordinates by unprojection and established the two line equations. 
       Then to find the shortest distance between these 2 lines, we can create two constraints that this line would be orthogonal to both the lines. 
       This gives rise to two equations with 2 unknowns. Once these unknowns are computed, I found the points on these lines that are the closest to each other 
       and their midpoint gives us the answer.
5. To test if the estimated 3D world coordinate is close enough, I reprojected the computed 3D coordinates onto the image planes using the computed projection 
   matrices and calculated the reprojection error.