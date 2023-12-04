# RGB-D Camera Trajectory Recovery

This project (for CS 678 - Computer Vision) focuses on reconstructing the unrestricted trajectory of an RGB-D camera by processing a sequence of consecutive RGB-D images captured while the camera is affixed to a drone observing a scene. The trajectory comprises poses, each defined by six degrees of freedom encompassing position and orientation, delineating the camera's movement over time. Through aligning each image $\(I_{t}\)$ with its preceding counterpart $\(I_{t - 1}\)$, the incremental computation of the camera's pose is achieved. This process involves determining the optimal rigid transformation $\(T_t\)$ that aligns $\(I_t\)$ with $\(I_{t - 1}\)$, consequently expressing the camera's pose as:

$X_t = T_tX_{t - 1}$

This equation assumes the initial pose originates at the scene's origin.

The RGB-D camera provides color and depth data for scene points. Utilizing this information along with camera calibration properties facilitates the creation of a 3D point cloud at each time step. Employing the Iterative Closest Point algorithm (ICP) [Tykkälä et al., 2011], successive 3D point clouds are registered to determine the transformation between them. This is achieved by minimizing the distance between corresponding points in the two point clouds.
