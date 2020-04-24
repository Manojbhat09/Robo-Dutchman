import numpy as np

class PurePursuit(object):
    def __init__(self, start, end, lookahead=0.05, dist_thresh=0.05):
        self.start = start
        self.end = end
        self.l = lookahead
        self.dist_thresh = dist_thresh

    def get_ang_vel(self, state, v):
        # return 0 if close to end
        x, y, _ = state
        dist = np.sqrt((self.end[1] - y)**2 + (self.end[0] - x)**2)
        if dist < self.dist_thresh:
            return 0
        
        # find lookahead point
        r, point = self.get_lookahead(state)
        
        # convert to robot frame
        point_r = self.transform(point, state)

        # find curvature
        # https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
        curvature = 2.0 * point_r[1] / (r * r)

        # return [v, w]
        # https://courses.lumenlearning.com/physics/chapter/6-1-rotation-angle-and-angular-velocity/
        # w / v = curvature -> w = v * curvature
        w = v * curvature

        return w

    # https://mathworld.wolfram.com/Circle-LineIntersection.html
    def get_lookahead(self, state):
        xr, yr, _ = state        
        x1, y1 = self.start
        x2, y2 = self.end

        x1 -= xr
        x2 -= xr
        y1 -= yr
        y2 -= yr

        r = self.l

        dx = x2 - x1
        dy = y2 - y1
        dr = np.sqrt(dx**2 + dy**2)
        D = x1 * y2 - x2 * y1

        delta = (r ** 2) * (dr ** 2) - D ** 2

        while delta < 0:
            r += 0.01
            delta = (r ** 2) * (dr ** 2) - D ** 2

        if delta == 0:
            ix = D * dy / (dr ** 2)
            iy = -D * dx / (dr ** 2)

            return [r, [ix + xr, iy + yr]]
        else:
            ix1 = (D * dy + self.sign(dy) * dx * np.sqrt(delta)) / (dr ** 2)
            iy1 = (-D * dx + np.abs(dy) * dx * np.sqrt(delta)) / (dr ** 2)

            ix2 = (D * dy - self.sign(dy) * dx * np.sqrt(delta)) / (dr ** 2)
            iy2 = (-D * dx - np.abs(dy) * dx * np.sqrt(delta)) / (dr ** 2)

            dist1 = (iy1 - y2) ** 2 + (ix1 - x2) ** 2
            dist2 = (iy2 - y2) ** 2 + (ix2 - x2) ** 2
            distr = x2 ** 2 + y2 ** 2

            if distr < dist1 and distr < dist2:
                return [r, [x2 + xr, y2 + yr]]
            if dist1 < dist2:
                return [r, [ix1 + xr, iy1 + yr]]
            else:
                return [r, [ix2 + xr, iy2 + yr]]


    def sign(self, x):
        if x < 0: return -1
        else: return 1

    # https://mathworld.wolfram.com/RotationMatrix.html
    def transform(self, point, state):
        x, y, th = state

        R = np.array(
            [
                [np.cos(th), -np.sin(th), x],
                [np.sin(th),  np.cos(th), y],
                [0,           0,          1]
            ]
        )

        x_w = np.array([point[0], point[1], 1.0]).reshape((-1,1))
        x_r = np.linalg.inv(R).dot(x_w).reshape(-1)
        
        return list(x_r[:2])