<h1 align="center">Sphere</h1>
<h3 align="left">simple code to create a point cloud of a sphere as an object, given the following parameters as input:</h3>

- center point for the sphere: [x, y, z]
- Radious of the sphere: R
- Rotation axis direction: [rx, ry, rz]
- division of the meshgrid in deg, for 1 degree it creates a circel with 360 points, for 30 degree it creates a
- circle represented by 12 points and so on.

<h3 align="left">Outputs:</h3>

- 3D points representing the Sphere: array[m x 3]
- normal vectors corresponding to each point array[m x 3]

<h3 align="left">Object features:</h3>
- Sphere.translate(point): Translate the point cloud to a given point, if **kwrds Relative=false, it moves the sphere center to the given point,
if false then it moves on each axis the given value on the point (considered as a moving vector).
- Sphere.rotate(Vector): alligns the sphere axis with the given vectro direction.

<h3 align="left">Example code:</h3>







