import numpy as np

def deg_to_rad(theta_deg):
    return (np.pi * theta_deg) / 180.0

def rot_x(theta_deg):
    '''
    Computes 3D rotation matrix about X axis.
    '''
    cos_rad= np.cos(deg_to_rad(theta_deg))
    sin_rad= np.sin(deg_to_rad(theta_deg))
    R= np.array([[1, 0, 0],
                 [0, cos_rad, -sin_rad],
                 [0, sin_rad, cos_rad]])
    return R

def rot_z(theta_deg):
    '''
    Computes 3D rotation matrix about Z axis.
    '''
    cos_rad= np.cos(deg_to_rad(theta_deg))
    sin_rad= np.sin(deg_to_rad(theta_deg))
    R= np.array([[cos_rad, -sin_rad, 0],
                 [sin_rad, cos_rad, 0],
                 [0, 0, 1]])
    return R

def compute_extrinsic_matrix(R_c,C):
    '''
    Returns the 3x4 extrinsic matrix from the camera pose.

    Inputs:
    3x3 rotation matrix describing the camera's orientation wrt world
    3x1 column vector describing the camera's location wrt world

    '''
    E= np.zeros((3,4))
    E[0:3,0:3]= np.transpose(R_c)
    E[0:3,3]= -np.matmul(np.transpose(R_c), C)
    return E

def compute_intrinsic_matrix(f,w,h):
    return np.array([[f, 0, w/2.0],
                     [0, f, h/2.0],
                     [0, 0, 1]])

def create_A(Ps,xs):
    '''
    Computes 4x4 matrix A that contains the coefficients of the linear system Ax=0.

    Inputs:
    Ps- array of camera projection matrices at locations 1 and 2
    xs- array of pixel coordinates in both images

    '''
    A=[]
    for i in range(len(xs)):
        proj_mat= Ps[i]
        icoorx,icoory= xs[i]
        A.append(icoorx * proj_mat[2,:] - proj_mat[0,:])
        A.append(icoory * proj_mat[2,:] - proj_mat[1,:])
    A= np.array(A)
    return A

def main():

    # Given
    f= 270.0
    w= 640.0
    h= 480.0
    theta1= -20.0
    theta2= 30.0
    cam1_center_w= np.array([5.0, 2.0, 1.0])
    cam2_center_w= np.array([8.5, 2.5, 2.0])
    u1= 360.0
    v1= 300.0
    u2= 140.0
    v2= 404.0

    # Compute camera intrinsic matrix ('K', dimension: 3x3)
    K= compute_intrinsic_matrix(f,w,h)

    # Compute camera extrinsic matrix at location 1 ('E1', dimension: 3x4) ######
    R_w_from_cam1= np.matmul(rot_z(theta1),rot_x(-90.0))
    t_w_from_cam1= cam1_center_w
    R_cam1_from_w= np.transpose(R_w_from_cam1)
    t_cam1_from_w= -np.matmul(R_cam1_from_w, t_w_from_cam1)
    E1= compute_extrinsic_matrix(R_w_from_cam1,t_w_from_cam1)
    # print(E1,"\n")

    # Compute camera extrinsic matrix at location 2 ('E2', dimension: 3x4) ######
    R_w_from_cam2= np.matmul(rot_z(theta2),rot_x(-90.0))
    t_w_from_cam2= cam2_center_w
    R_cam2_from_w= np.transpose(R_w_from_cam2)
    t_cam2_from_w= -np.matmul(R_cam2_from_w, t_w_from_cam2)
    E2= compute_extrinsic_matrix(R_w_from_cam2, t_w_from_cam2)
    # print(E2,"\n")

    # Sanity test for extrinsics translation (t vs -R.T * t)
    # cam1 and cam2 origins are (0,0,0) in respective camera coordinate frames. Convert to world coordinates and check if we get the given cam centers.
    origin= np.array([0, 0, 0])
    cam1_center_w_= np.matmul(R_w_from_cam1, origin) + t_w_from_cam1
    cam2_center_w_= np.matmul(R_w_from_cam2, origin) + t_w_from_cam2
    print("Testing camera extrinsics...")
    print(f"cam1_center_w_: {cam1_center_w_}")
    print(f"cam2_center_w_: {cam2_center_w_}\n")

    # Sanity test for extrinsics rotation and unprojection
    # Unproject center pixel of each camera. This should give 0,0,1 ray in the respective camera coordinates.
    # cam1
    uv_center1= np.array([w/2, h/2, 1.0])
    principal_ray_cam1= np.matmul(np.linalg.inv(K), uv_center1)
    print("Testing camera 1 unprojection...")
    print(f"principal_ray_cam1: {principal_ray_cam1}")
    principal_ray_w= np.matmul(R_w_from_cam1, principal_ray_cam1)
    print(f"principal_ray_w: {principal_ray_w}\n")

    uv_below_center1= np.array([w/2, h/2 + h/4, 1.0])
    ray_below_cam1= np.matmul(np.linalg.inv(K), uv_below_center1)
    print(f"ray_below_cam1: {ray_below_cam1}")
    ray_below_w= np.matmul(R_w_from_cam1, ray_below_cam1)
    print(f"ray_below_w: {ray_below_w}\n")

    uv_above_center1= np.array([w/2, h/2 - h/4, 1.0])
    ray_above_cam1= np.matmul(np.linalg.inv(K), uv_above_center1)
    print(f"ray_above_cam1: {ray_above_cam1}")
    ray_above_w= np.matmul(R_w_from_cam1, ray_above_cam1)
    print(f"ray_above_w: {ray_above_w}\n")

    uv_left_center1= np.array([w/2 - w/4, h/2, 1.0])
    ray_left_cam1= np.matmul(np.linalg.inv(K), uv_left_center1)
    print(f"ray_left_cam1: {ray_left_cam1}")
    ray_left_w= np.matmul(R_w_from_cam1, ray_left_cam1)
    print(f"ray_left_w: {ray_left_w}\n")

    uv_right_center1= np.array([w/2 + w/4, h/2, 1.0])
    ray_right_cam1= np.matmul(np.linalg.inv(K), uv_right_center1)
    print(f"ray_right_cam1: {ray_right_cam1}")
    ray_right_w= np.matmul(R_w_from_cam1, ray_right_cam1)
    print(f"ray_right_w: {ray_right_w}\n")

    # cam2
    uv_center2= np.array([w/2, h/2, 1.0])
    print("Testing camera 2 unprojection...")
    principal_ray_w= np.matmul(R_w_from_cam2, principal_ray_cam1)
    print(f"principal_ray_w: {principal_ray_w}\n")

    # Compute camera projection matrices at locations 1 and 2 ('P1, P2', dimension: 3x4) ######
    P1= np.matmul(K, E1)
    P2= np.matmul(K, E2)
    print(f"Projection matrix for location 1: {P1}\n")
    print(f"Projection matrix for location 2: {P2}\n")

    # Sanity test to check if camera center is the null space of the projection matrix
    if(np.all(np.isclose(np.matmul(P2, np.array([8.5, 2.5, 2.0, 1.0])), np.zeros((1, 3)))) and np.all(np.isclose(np.matmul(P1, np.array([5.0, 2.0, 1.0, 1.0])), np.zeros((1, 3))))):
        print("Given camera centers tested to be the null spaces of respective projection matrices. \n")

    turtle_position = np.array([0.0, 0.0, 0.0])

    # Method 1- linear triangulation using DLT
    # Compute A for Ax=0
    print("Method 1: Performing linear triangulation...")
    Ps= np.array([P1, P2])
    xs= np.array([np.array([u1, v1]), np.array([u2, v2])])
    A= create_A(Ps,xs)

    # Perform singular value decomposition
    _,_,V_transpose= np.linalg.svd(A)
    turtle_position_homogeneous= V_transpose[-1]

    # Normalize homogeneous coordinates
    turtle_position_homogeneous/= turtle_position_homogeneous[3]
    turtle_position= turtle_position_homogeneous[:3]
    print(f"The turtle is at {turtle_position}.\n")

    # Method 2- midpoint triangulation
    print("Method 2: Performing midpoint triangulation...")
    # Find the 2 rays passing through the pixel coordinates
    ray1_cam1= np.matmul(np.linalg.inv(K), np.array([u1, v1, 1.0]))
    ray2_cam2= np.matmul(np.linalg.inv(K), np.array([u2, v2, 1.0]))
    ray1_w= np.matmul(R_w_from_cam1, ray1_cam1)
    ray2_w= np.matmul(R_w_from_cam2, ray2_cam2)
    # print(ray1_w, ray2_w)

    # Formulate system of equations and compute scalars
    # (f1 - f2).T * ray1_w= 0 and (f1 - f2).T * ray2_w = 0
    # Equation 1: c1 + (lambda1 * a1) - (lambda2 * b1) = 0 and 
    # Equation 2: c2 + (lambda1 * a2) - (lambda2 * b2) = 0
    c1= np.matmul(np.transpose(cam1_center_w - cam2_center_w), ray1_w)
    c2= np.matmul(np.transpose(cam1_center_w - cam2_center_w), ray2_w)
    a1= np.matmul(np.transpose(ray1_w), ray1_w)
    a2= np.matmul(np.transpose(ray1_w), ray2_w)
    b1= -np.matmul(np.transpose(ray2_w), ray1_w)
    b2= -np.matmul(np.transpose(ray2_w), ray2_w)
    lambda2= (c2*a1 - c1*a2) / (b1*a2 - b2*a1)
    lambda1= (-c1 - (b1*lambda2)) / a1
    
    # Compute mid point
    f1= cam1_center_w + (lambda1 * ray1_w)
    f2= cam2_center_w + (lambda2 * ray2_w)
    turtle_position= (f1 + f2) / 2.0
    print(f"The turtle is at {turtle_position}.\n")

    # Sanity test to reproject triangulated 3D coordinate back to image planes using projection matrices
    print("Computing reprojection error...")
    uv1_homogeneous= np.matmul(P1, np.transpose(np.append(turtle_position, 1.0)))
    uv1= uv1_homogeneous / uv1_homogeneous[2]
    uv2_homogeneous= np.matmul(P2, np.transpose(np.append(turtle_position, 1.0)))
    uv2= uv2_homogeneous / uv2_homogeneous[2]
    print(f"uv1_: {uv1}")
    print(f"uv2_: {uv2}")
    print(f"Reprojection error in pixels in cam1: {uv1 - np.array([u1, v1, 1.0])}")
    print(f"Reprojection error in pixels in cam2: {uv2 - np.array([u2, v2, 1.0])}")
    
if __name__ == "__main__":
    main()