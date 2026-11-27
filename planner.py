
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 

        #NOTE: laster_sig should be tuned based on the buffer we need from obstacles. starting value of 0.33 is allowing for the 34cm diameter robot (0.17m radius) + 5cm of clearance from obstacles. tune based on the positiion and tracking error of the robot.
        self.m_utilites = mapManipulator(
            filename_=self.mapName,
            laser_sig=0.33,
        )
        self.costMap = self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # Reject invalid endpoints before searching the entire map.
        height, width = self.costMap.shape
        for label, cell in [("Start", startPose), ("Goal", endPose)]:
            if not (0 <= cell[0] < width and 0 <= cell[1] < height):
                raise ValueError(f"{label} is outside the map")
        if self.costMap[endPose[1], endPose[0]] > 0.8:
            raise ValueError("Goal is inside an obstacle or its clearance buffer")

        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        path_cells = search(self.costMap, startPose, endPose)
        if not path_cells or path_cells[-1] != endPose:
            raise ValueError("A* could not find a complete path to the goal")
        path = [list(self.m_utilites.cell_2_position(cell)) for cell in path_cells]

        # TODO PART 5 return the path as list of [x,y]
        return path

if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()

    # you can use this part of the code to test your 
    # search algorithm regardless of the ros2 hassles
    
