import open3d as o3d
import numpy as np
from sklearn import preprocessing

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    # print(a, b)
    v = np.cross(a, b)
    if any(v): #if not all zeros then
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))

    elif a[0] == b[0] and a[1] == b[1] and a[2] == b[2]:
        return np.eye(3)  # cross of all zeros only occurs on identical directions
    else:
        return -1 * np.eye(3) #cross of all zeros only occurs on identical directions


class Sphere:
    def __init__(self, center: np.array, axisDirect: np.array, SphereRadious: int, AngleDivision: float):
      
        if np.shape(center) == (3,):
            self.center = center
        else:
            raise Exception('center must be  a point of 1x3 size')
        if np.shape(axisDirect) == (3,):
            self.axisDirect = axisDirect
        else:
            raise Exception('center must be  a point of 1x3 size')

        self.SphereRadious = SphereRadious
        self.AngleDivision = AngleDivision
        self.points = self.createSphere()
        self.normals = self.createNormals()

    def getRadious(self):
        return self.SphereRadious

    def getPoints(self):
        return self.points

    def getNormals(self):
        return self.normals

    def createSphere(self):
        R = self.SphereRadious
        num_of_points = 360 / self.AngleDivision
        num_of_points2 = num_of_points/2
        u, v = np.mgrid[0:np.pi:(num_of_points/2 + 1) * 1j, 0:2*np.pi:((num_of_points + 1)) * 1j]
        x = np.cos(u) * np.sin(v)
        y = np.sin(u) * np.sin(v)
        z = np.cos(v)
        points = np.zeros([0, 3])
        points_circle = {}
        for i in range(len(x)):
            points1 = np.zeros([0, 3])
            for j in range(len(x[0])):
                POINT = [x[i][j] * R, y[i][j] * R, z[i][j] * R]
                points1 = np.vstack([points1, POINT])
            points_circle[i] = points1
            points = np.vstack([points, points1])
            # translate points to center
        points[:, 0] = points[:, 0] - self.center[0]
        points[:, 1] = points[:, 1] - self.center[1]
        points[:, 2] = points[:, 2] - self.center[2]
        return points

    def createNormals(self):
      
        point2 = np.array(self.points)
        normals1 = np.ones([np.shape(self.points)[0], 3])
        normals1[:, 0] = (normals1[:, 0] * point2[:, 0]) + self.center[0]
        normals1[:, 1] = (normals1[:, 1] * point2[:, 1]) + self.center[1]
        normals1[:, 2] = (normals1[:, 2] * point2[:, 2]) + self.center[2]
        normals = preprocessing.normalize(normals1, norm='l2')
        return normals

    def rotate(self, new_axis: np.array):
        if np.shape(new_axis) == (3,):
            ROT = rotation_matrix_from_vectors(new_axis, self.axisDirect)
            pcd_act = o3d.geometry.PointCloud()
            pcd_act.points = o3d.utility.Vector3dVector(np.asarray(self.points))
            pcd_act.rotate(ROT.T, center=self.center)
            self.center = pcd_act.get_center()
            self.points = np.asarray(pcd_act.points)
            pcd_act.clear()
        else:
            raise Exception('point must be must be  an array with shape [1,3]')

    def translate(self, point: np.array, Relative=False, **kwargs):
        if np.shape(point) == (3,):
            move2 = np.array(self.center) - np.array(point)
            if Relative == False:
                self.points[:, 0] = self.points[:, 0] + move2[0]
                self.points[:, 1] = self.points[:, 1] + move2[1]
                self.points[:, 2] = self.points[:, 2] + move2[2]
                self.center = point

            else:
                self.points[:, 0] = self.points[:, 0] + point[0]
                self.points[:, 1] = self.points[:, 1] + point[1]
                self.points[:, 2] = self.points[:, 2] + point[2]
                self.center = self.center + point
            pass
        else:
            raise Exception('point must be must be  an array with shape [1,3]')

# test algorithm

sphere = Sphere([0, 0, 0],[0, 0, 1], 100, 5)
radious = sphere.getRadious()
sphere.translate([20,20,50]) # move the sphere center to the given point
sphere.rotate([1, 0, 0]) # align the sphere rotation axis to the given vector direction
points = sphere.getPoints()
normals = sphere.getNormals()
# create a o3d point cloud object to plot the sphere
pcd_act = o3d.geometry.PointCloud()
pcd_act.points = o3d.utility.Vector3dVector(points)
pcd_act.paint_uniform_color([1, 0, 0])
# todo: plot normal vectors
# pcd_act2 = o3d.geometry.PointCloud()
# pcd_act2.points = o3d.utility.Vector3dVector(points2 + 10*normals)
# pcd_act2.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([pcd_act])
