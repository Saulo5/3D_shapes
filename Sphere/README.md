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

- Sphere.rotate(Vector): alligns the sphere axis with the given vector direction

<h3 align="left">Example code:</h3>

`sphere = Sphere([0, 0, 0],[0, 0, 1], 100, 5)`

`points_sphere = sphere.getPoints()`

`normals_sphere = sphere.getNormals()`

-Transform point cloud sphere to point [20,50,100], and align rotation axis axis to the following direction [1, 0, 0]. 

`sphere.translate([20,50,100]) # move the sphere center to the given point`

`sphere.rotate([1, 0, 0]) # align the sphere rotation axis to the given vector direction`

`points_transf = sphere.getPoints()`

`normals_transf = sphere.getNormals()`

-create a o3d point cloud object to plot the sphere

`pcd_1 = o3d.geometry.PointCloud()`

`pcd_1.points = o3d.utility.Vector3dVector(points_sphere)`

`pcd_1.paint_uniform_color([1, 0, 0]) # Red for original sphere`

`pcd_2 = o3d.geometry.PointCloud()`

`pcd_2.points = o3d.utility.Vector3dVector(points_transf)`

`pcd_2.paint_uniform_color([0, 0, 1]) # Blue for translated`

`o3d.visualization.draw_geometries([pcd_1, pcd_2])`


![screen-gif](./data/spheres.gif)









