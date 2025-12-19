from dataclasses import dataclass
import cv2

@dataclass(frozen=True)
class Points:
    x_min: int
    x_max: int
    y_min: int
    y_max: int
    
    @property
    def w(self) -> int:
        return self.x_max - self.x_min

    @property
    def h(self) -> int:
        return self.y_max - self.y_min

def project_point(pt3d, fx, fy, cx, cy):
    print(pt3d)
    x, y, z = pt3d
    if z == 0:
        return None
    u = int(fx * x / z + cx)
    v = int(fy * y / z + cy)
    return (u, v)

def draw_projected_arrow_cv(img, normal_vector, centroid, fx, fy, cx, cy, scale=0.05, color=(255,0,0)):
    def project(pt3d):
        print(pt3d)
        x, y, z = pt3d
        if z == 0:
            return None
        u = int(fx * x / z + cx)
        v = int(fy * y / z + cy)
        return (u, v)

    if centroid is not None:
        start_2D = project(centroid)
        arrow_end_3D = centroid + scale * normal_vector
        end_2D = project(arrow_end_3D)

        cv2.arrowedLine(img, start_2D, end_2D, color, 2, tipLength=0.05)
    return start_2D
