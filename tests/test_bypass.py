"""Physical detour geometry, blocked shared gates and impossible corridors."""
import json, math, unittest
from pathlib import Path
import numpy as np
from intramural_mission.bypass import plan, winding
from intramural_mission.route import segments, clearance
C = json.loads((Path(__file__).resolve().parents[1]/'config/arena.json').read_text())


def box(x,y):
    return [(a,b) for a in np.arange(x-.25,x+.251,.05) for b in np.arange(y-.25,y+.251,.05)]


class BypassTests(unittest.TestCase):
    def test_four_phases_keep_winding_and_collision_clearance(self):
        for index, target in ((0,(1.5,.75)),(1,(2.44,-.06)),(4,(2.44,2.94)),(3,(3.5,2.5)),(7,(3.5,2.5))):
            with self.subTest(index=index):
                route=segments(C)
                progress=3 if index==0 else 0
                pose=route[index][1][progress]
                result=plan(C,route,index,progress,pose,box(*target))
                self.assertIsNotNone(result)
                route[index:index+len(result)]=[(result[0][0],route[index][1][:progress]+result[0][1])]+result[1:]
                path=[p for _,points in route for p in points]
                self.assertEqual([winding(path,C[k]) for k in ('lower_pole','upper_pole')],[1,-1])
                for _,points in result:
                    for p in points:
                        self.assertGreater(min(clearance(C,p)),.15)
                        distance=math.hypot(max(abs(p[0]-target[0])-.25,0),max(abs(p[1]-target[1])-.25,0))
                        self.assertGreater(distance,.45)
                if index in (3,7):
                    self.assertEqual(len(result),2)
                    self.assertEqual(result[0][1][-1],result[1][1][0])

    def test_no_path_does_not_create_route_or_move_parking(self):
        route=segments(C)
        obstacles=[(x,y) for x in np.arange(-1.5,6.5,.05) for y in np.arange(-.5,1.51,.05)]
        self.assertIsNone(plan(C,route,0,0,C['parking'],obstacles))
        self.assertIsNone(plan(C,route,9,0,route[9][1][0],box(*C['parking'])))
