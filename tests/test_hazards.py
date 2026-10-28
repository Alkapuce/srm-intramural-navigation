"""Ordered-segment obstacle gate and coordinate transforms."""

import unittest

from intramural_mission.hazards import first_blocked


class HazardTests(unittest.TestCase):
    def test_vectorized_sweep_matches_independent_cell_walk(self):
        import math
        import numpy as np
        rng=np.random.default_rng(42)
        for _ in range(100):
            width,height=50,40;resolution=.1;radius=float(rng.uniform(.1,.4))
            grid=rng.choice([0,100],width*height,p=[.995,.005]).tolist()
            points=np.cumsum(rng.normal(0.,.04,(45,2)),axis=0)+[2.,2.]
            progress=int(rng.integers(0,20));position=points[progress]+rng.normal(0.,.02,2)
            tx,ty,yaw=rng.uniform(-.5,.5,3);ox,oy,oyaw=rng.uniform(-.5,.5,3)
            reach=0.;previous=position;expected=None
            for i in range(progress,len(points)):
                x,y=points[i];reach+=math.dist(previous,(x,y));previous=(x,y)
                if reach>1.2:break
                gx=math.cos(yaw)*x-math.sin(yaw)*y+tx-ox
                gy=math.sin(yaw)*x+math.cos(yaw)*y+ty-oy
                ix=math.floor((math.cos(oyaw)*gx+math.sin(oyaw)*gy)/resolution)
                iy=math.floor((-math.sin(oyaw)*gx+math.cos(oyaw)*gy)/resolution)
                n=math.ceil(radius/resolution)
                if any(0<=iy+dy<height and 0<=ix+dx<width and math.hypot(dx,dy)*resolution<=radius
                       and grid[(iy+dy)*width+ix+dx]>=90 for dy in range(-n,n+1) for dx in range(-n,n+1)):
                    expected=i;break
            self.assertEqual(first_blocked(points,progress,position,grid,width,height,resolution,
                (ox,oy,oyaw),(tx,ty,yaw),radius=radius),expected)

    def test_obstacle_ahead_blocks_but_behind_or_other_segment_does_not(self):
        width = height = 60
        resolution = 0.1
        grid = [0] * (width * height)
        grid[20 * width + 20] = 100
        points = [(i / 10, 2.0) for i in range(10, 42)]
        self.assertIsNotNone(first_blocked(points, 0, (1, 2), grid, width, height,
                                           resolution, (0, 0, 0), radius=.2))
        self.assertIsNone(first_blocked(points, 20, (3, 2), grid, width, height,
                                        resolution, (0, 0, 0), radius=.2))
        self.assertIsNone(first_blocked([(i / 10, 4) for i in range(10, 42)], 0,
                                        (1, 4), grid, width, height, resolution,
                                        (0, 0, 0), radius=.2))

    def test_map_to_rolling_odom_frame(self):
        width = height = 60
        grid = [0] * (width * height)
        grid[20 * width + 30] = 100
        # Map path x=2 becomes odom x=3 after map->odom +1 m.
        self.assertIsNotNone(first_blocked([(2, 2)], 0, (1.9, 2), grid, width,
                                            height, .1, (0, 0, 0), (1, 0, 0),
                                            radius=.1))

    def test_invalid_grid_rejected(self):
        with self.assertRaises(ValueError):
            first_blocked([], 0, (0, 0), [], 10, 10, .1, (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
