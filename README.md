# Motion Planning in Dynamic Environments using Dynamic Roadmaps
End-to-end implementation of the Dynamic Roadmaps (DRM) algorithm for motion planning in dynamic environments. The code utilizes the **Grapeshot API** for interaction with **PyBullet** using **OMPL**, developed by the [ELPIS Lab](https://elpislab.org/) (not publicly available). This project was completed as part of the **Motion Planning (RBE 550)** course at **Worcester Polytechnic Institute (WPI)**, taught by Professor Constantinos Chamzas.
The report uploaded to this repository shows details of the implementation and its related theory.

## Main Features
- Implementation of the DRM algorithm for real-time path planning in dynamic environments using OMPL with Python.
- Implements path repair using RRTConnect for disconnected roadmaps.
- Utilizes PyBullet for visualization and simulation.

## Workflow of the Algorithm
- **Roadmap Construction**: Uses PRM to build an initial roadmap in an obstacle-free environment.
- **Dynamic Obstacle Integration**: Checks roadmap nodes and edges for collisions with dynamic obstacles.
- **Roadmap Updates**: Removes invalid nodes and edges while maintaining connectivity.
- **Pathfinding**: Finds the shortest path from start to goal in the updated roadmap using Dijkstra's algorithm.
- **Visualization**: Visualizes the roadmap and path in PyBullet.

## Results
- The algorithm outperformed the naive approach in removing vertices and edges significantly, with DRM taking ~0.2s compared to the naive approach, which took ~5s with 5s of roadmap construction.
- The DRM algorithm was implemented on the **Fetch Robot**, and the output of the planning can be seen in the visualization shown below:

<p align='center'>
    <img src="DRM_Final_Demo_3.gif" alt="drawing" width="800"/>
</p>


## Future Work/ TODO
- Add code to save the discretization performed in the environment so that it can be loaded every time an obstacle is added to/removed from the environment. Ideally, the discretization should be done offline, and the planning should be done when the obstacles are added to the environment.
- Implement A* algorithm for graph search.
- Add provision in code for tracking obstacle addition/removal.

## References
This implementation is based on the following papers:
- Leven, Peter, and Seth Hutchinson. “A framework for real-time path planning in changing environments.” The International Journal of Robotics Research 21.12 (2002): 999-1030. [Link](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=5987ec6a38fc2bcb10f9137dbdcba6ec99735989)
- Kallman, M., and M. Mataric. “Motion planning using dynamic roadmaps.” IEEE International Conference on Robotics and Automation (2004): 4399-4404. [Link](http://graphics.ucmerced.edu/publications/2004_ICRA_Kallmann.pdf)
- Yang, Y., et al. "HDRM: A Resolution Complete Dynamic Roadmap for Real-Time Motion Planning in Complex Scenes." IEEE Robotics and Automation Letters 3.1 (2018): 551-558. [Link](https://ieeexplore.ieee.org/document/8110660)
