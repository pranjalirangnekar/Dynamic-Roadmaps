"""Planning for MotionBenchMaker problems.
"""
from fire import Fire

from grapeshot.assets import ROBOTS
from grapeshot.assets.environments import PLANE, TABLE_URDF, CUBE, CUBE_2
from grapeshot.extensions.octomap import load_mbm_octomap
from grapeshot.model.world import World
from grapeshot.model.robot import process_srdf
from grapeshot.planning.context import get_OMPL_context, get_OMPL_statespace, OMPLGeometric
from grapeshot.planning.goals import getJointBoundGoal
from grapeshot.planning.moveit import process_moveit_request
from grapeshot.planning.trajectory import path_to_trajectory
from grapeshot.simulators.pybullet import PyBulletSimulator
from grapeshot.model.simulator import ControlMode
from grapeshot.model.skeleton import JointType
from grapeshot.util.tf import Pose
from grapeshot.util.random import RNG
import ompl.geometric._geometric as og
import ompl.base._base as ob
import numpy as np
import time
import networkx as nx
from ompl.geometric import PathGeometric
import csv
import time


def shortest_path_in_roadmap(updated_roadmap, si, start_state, goal_state):
    # Build a state->index map and Nx graph
    nx_graph_sub = nx.Graph() 
    state_map = {}
    for i in range(updated_roadmap.numVertices()):
        st = updated_roadmap.getVertex(i).getState()
        nx_graph_sub.add_node(i)
        state_map[i] = st

    for i in range(updated_roadmap.numVertices()):
        for j in range(i + 1, updated_roadmap.numVertices()):
            if updated_roadmap.edgeExists(i, j):
                nx_graph_sub.add_edge(i, j)

    start_idx = None
    goal_idx = None
    for i in range(updated_roadmap.numVertices()):
        if si.getStateSpace().equalStates(state_map[i], start_state):
            start_idx = i
        if si.getStateSpace().equalStates(state_map[i], goal_state):
            goal_idx = i

    if start_idx is None or goal_idx is None:
        print("Could not find start or goal state in updated roadmap for shortest path.")
        return None

    try:
        path_indices = nx.shortest_path(nx_graph_sub, start_idx, goal_idx)
        path_geom = PathGeometric(si)
        for idx in path_indices:
            path_geom.append(state_map[idx])
        return path_geom
    except nx.NetworkXNoPath:
        return None


def store_roadmap_vertices(roadmap,skel_robot,context,request,world,file_path,robot):

    end_effector_link_name = ""

    if robot == "fetch":
        end_effector_link_name = "r_gripper_finger_link"
    else:
        end_effector_link_name = "panda_finger_joint1"


    eef_link = None


    for i in skel_robot.acm.links():
        
        link = skel_robot.get_link(i[0])
        
        if link is not None:
            if link.name == end_effector_link_name:
                eef_link = link


    graph_coordinates = []

    for _idx in range(roadmap.numVertices()):
       
        vertex_state = roadmap.getVertex(_idx).getState()

        joint_values = context.state_to_array(vertex_state)
        joint_indices = request.group.indices

        # Map joint_values to a dict {joint: [value]} for set_joint_positions()
        joint_map = {}
        for joint in request.group.joints:
            idx = joint_indices[joint][0]  # since we expect 1D joints here
            joint_map[joint] = [joint_values[idx]]

        # Set the joint positions in the world
        world.set_joint_positions(joint_map)

        # Do forward kinematics
        world.fk()

        # Get end-effector pose
        eef_pose = world.get_pose(eef_link)
        x, y, z = eef_pose.position

        graph_coordinates.append((x,y,z))

   

    # Write to CSV
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z"])  # Add a header row
        writer.writerows(graph_coordinates)

    print(f"Coordinates have been saved to {file_path}.")
    



def main(
        robot: str = "fetch",          # Robot to load. See assets/ folder for all robots.
        problem: str = "table_pick",   # Problem to solve. See assets/ folder for problems availble for robots.
        planner: str = "PRM",           # Planner to use. Anything in the ompl.geometric namespace.
        load_plane: bool = False,      # Load the ground plane.
        visualize: bool = False,       # Visualize with GUI.
        simplify: bool = False,        # Simplify the path.
        resolution: float = 0.003,     # Validity checking resolution for planning.
        speedup: float = 1,            # Speedup to animation when displaying
        benchmark: bool = False,       # Rather than planning, benchmark this planning problem.
        runs: int = 10,                # Number of runs to use in benchmarking.
        sensed: bool = False,          # Use the sensed representation of the problem if available.
        show_goal: bool = False        # Show goal state
    ):
                                       # Get the requested robot and problem
   
    
    
    
    robot_resource_tmp = ROBOTS[robot]
    problem_resource_tmp = robot_resource_tmp.problems[problem]
    world_tmp = World(PyBulletSimulator(True))
    skel_tmp = world_tmp.add_skeleton(robot_resource_tmp.urdf)
    groups_tmp = process_srdf(skel_tmp, robot_resource_tmp.srdf)


    # Add table
    table_skel = world_tmp.add_skeleton(problem_resource_tmp.environment, self_collision = False)
    

    world_tmp.setup_collision_filter()

    # Load motion planning request from MotionBenchMaker's format (MoveIt MotionPlanningRequest message in YAML)
    request_tmp = process_moveit_request(skel_tmp, groups_tmp, problem_resource_tmp.request)

    # Set state to initial from request.
    world_tmp.set_joint_positions(request_tmp.initial_state)

    

    # Get planning context for specified planning group.
    context_tmp = get_OMPL_context(world_tmp, [request_tmp.group], "PRM")
    # context.planner.setRange(0.2)

    # Get starting scoped state from current world state.
    start_tmp = context_tmp.scoped_state_from_world(world_tmp)

    # Create goal from request and sample a goal inside.
    goal_state_tmp = context_tmp.scoped_state()
    goal_tmp = getJointBoundGoal(context_tmp, request_tmp)
    goal_tmp.sampleGoal(goal_state_tmp())

    context_tmp.set_state(start_tmp(), world_tmp)

    context_tmp.clear_query()

    # Set start and goal state.
    context_tmp.setup.setStartAndGoalStates(start_tmp, goal_state_tmp)


    # Using solve(), we add the start state to the roadmap and then we grow the roadmap
    print("Constructing obstacle-free roadmap .... ")
    context_tmp.planner.setProblemDefinition(context_tmp.pdef)
    context_tmp.planner.setup()
    ptc2 = ob.timedPlannerTerminationCondition(20.0)
    context_tmp.planner.solve(ptc2)

    ptc3 = ob.timedPlannerTerminationCondition(5.0)
    context_tmp.planner.constructRoadmap(ptc3)

    print(f"Number of vertices after solve and constructRoadmap = {context_tmp.get_planner_data().numVertices()}")
    
    # Extract the roadmap created in pldata
    pldata = ob.PlannerData(context_tmp.si)  
    context_tmp.planner.getPlannerData(pldata)
    print(f"Number of vertices, edges in plannerData object = {pldata.numVertices()} , {pldata.numEdges()}")
    

    print("Saving position of vertices of obstacle-free roadmap ...")
    if robot == "fetch":
        store_roadmap_vertices(pldata,skel_tmp,context_tmp,request_tmp,world_tmp,"obstacle_free_roadmap.csv",robot)

    context_tmp.set_state(start_tmp(), world_tmp)


    num_vertices = pldata.numVertices()
    num_edges = pldata.numEdges()
    
    
    print(f"Number of vertices before performing collision checking = {num_vertices}")
    print(f"Number of edges before performing collision checking = {num_edges}")
    

    # assign tags to vertices for identification and mappings. During graph modification, the vertex indices change
    for graph_ctr in range(num_vertices):
        pldata.getVertex(graph_ctr).setTag(graph_ctr)


    # Add obstacle in our cloned world to invalidate nodes
    (ss, organized_groups, set_state, get_state, state_to_array, array_to_state
        ) = get_OMPL_statespace([request_tmp.group], {})
    
    world_tmp.remove_skeleton(table_skel)

    empty_world_clone = world_tmp.create_headless_clone()
    

    vertex_mapping = {}
    edge_mapping = {}
    

    # cube used for discretization. This cube is moved in the workspace for mapping
    cube1 = empty_world_clone.add_skeleton(CUBE, name = f"cube1", base_type = JointType.FLOATING)
    
    
    ctr = 0
    ctr2 = 0

    # Set limits for discretization
    x_lim_s = 0.6 
    x_lim_e = 1.0 
    y_lim_s = 0.2 
    y_lim_e = 0.6 
    z_lim_s = 0.6 
    z_lim_e = 1.2
    step_s = 0.2

    
    print("Generating mapping ....")

    # generate descritization-based mapping for vertices 
    for x_dummy in np.arange(x_lim_s,x_lim_e,step_s):

        for y_dummy in np.arange(y_lim_s,y_lim_e,step_s):

            for z_dummy in np.arange(z_lim_s,z_lim_e,step_s):

                x_dummy= round(x_dummy,2)
                y_dummy = round(y_dummy,2)
                z_dummy = round(z_dummy,2)
                

                random = Pose(position = RNG.uniform(low = [x_dummy, y_dummy, z_dummy], high = [x_dummy, y_dummy, z_dummy], size = (3, )))
                empty_world_clone.set_joint_positions({cube1.base_joint: random.flat})
                for group in organized_groups:
                    empty_world_clone.set_group_control_mode(group, ControlMode.POSITION)

                setup = og.SimpleSetup(ss)
                dummy_context = OMPLGeometric(
                    empty_world_clone, organized_groups, set_state, get_state, state_to_array, array_to_state, setup
                    )
                
                dummy_context.si.setStateValidityCheckingResolution(1e-3)
                dummy_context.si.setup()

                for index in range(num_vertices):      
                    
                    roadmap_vertex = pldata.getVertex(index)
                    vertex_state = roadmap_vertex.getState()

                    if vertex_state is not None:
                        ctr+= 1
                        
                        if not dummy_context.si.isValid(vertex_state):
                        
                            if (x_dummy,y_dummy,z_dummy) in vertex_mapping:
                                vertex_mapping[(x_dummy,y_dummy,z_dummy)].append(roadmap_vertex.getTag())
                            else:
                                vertex_mapping[(x_dummy,y_dummy,z_dummy)] = []
                                vertex_mapping[(x_dummy,y_dummy,z_dummy)].append(roadmap_vertex.getTag())

                            ctr2+= 1


                
    empty_world_clone.remove_skeleton(cube1)

    # time.sleep(1.0)

    # Another cube used for mapping edges using discretization
    cube2 = empty_world_clone.add_skeleton(CUBE, name = f"cube2", base_type = JointType.FLOATING)

    # generate descritization-based mapping for vertices
    for x_in in np.arange(x_lim_s,x_lim_e,step_s):

        for y_in in np.arange(y_lim_s,y_lim_e,step_s):

            for z_in in np.arange(z_lim_s,z_lim_e,step_s):

                x_in = round(x_in,2)
                y_in = round(y_in,2)
                z_in = round(z_in,2)
                

                random_2 = Pose(position = np.array([x_in, y_in, z_in]))
                empty_world_clone.set_joint_positions({cube2.base_joint: random_2.flat})
                for group in organized_groups:
                    empty_world_clone.set_group_control_mode(group, ControlMode.POSITION)

                setup = og.SimpleSetup(ss)
                dummy_context_2 = OMPLGeometric(
                    empty_world_clone, organized_groups, set_state, get_state, state_to_array, array_to_state, setup
                    )
                
                dummy_context_2.si.setStateValidityCheckingResolution(1e-3)
                dummy_context_2.si.setup()

                motion_validator = ob.DiscreteMotionValidator(dummy_context_2.si)

                for i_1 in range(num_vertices):      

                    for i_2 in range(num_vertices):

                        roadmap_v_1 = pldata.getVertex(i_1)
                        roadmap_v_2 = pldata.getVertex(i_2)

                        if pldata.edgeExists(i_1,i_2):

                            if not motion_validator.checkMotion(roadmap_v_1.getState(),roadmap_v_2.getState()):
                                

                                if (x_in,y_in,z_in) in edge_mapping:
                                    edge_mapping[(x_in,y_in,z_in)].append((roadmap_v_1.getTag(),roadmap_v_2.getTag()))
                                else:
                                    edge_mapping[(x_in,y_in,z_in)] = []
                                    edge_mapping[(x_in,y_in,z_in)].append((roadmap_v_1.getTag(),roadmap_v_2.getTag()))
                    
    
    empty_world_clone.remove_skeleton(cube2)


    
    print("-------------------")

    print("Vertex map obtained :")
    print(vertex_mapping)

    print("------------------")

    print("Edge mapping :")
    print(edge_mapping)


    
    colliding_cells = []

    # Centre of obstacle
    x_c = 0.8 
    y_c = 0.4
    z_c = 0.8

    diff = step_s/2

    # distance checks
    d_x = 0.1 + diff
    d_y = 0.1 + diff
    d_z = 0.175 + diff

    # Perform intersection test to find colliding
    for x_d in np.arange(x_lim_s,x_lim_e,step_s):

        for y_d in np.arange(y_lim_s,y_lim_e,step_s):

            for z_d in np.arange(z_lim_s,z_lim_e,step_s):

                x_d= round(x_d,2)
                y_d = round(y_d,2)
                z_d = round(z_d,2)

                if ((abs(x_c - x_d)<=d_x) and (abs(y_c - y_d)<=d_y) and (abs(z_c - z_d)<=d_z)):

                    print(f"centre of cube colliding: ({x_d,y_d,z_d})")
                    colliding_cells.append((x_d,y_d,z_d))




    # Add table and obstacles back for visualization
    world_tmp.add_skeleton(problem_resource_tmp.environment, self_collision = False)
    cube3 = world_tmp.add_skeleton(CUBE_2, name = f"cube3", base_type = JointType.FLOATING)
    random_pose = Pose(position = np.array([x_c, y_c, z_c]))
    world_tmp.set_joint_positions({cube3.base_joint: random_pose.flat})
    
    
    print("Removing vertices and edges from mapping .....")
    start_time = time.time()

    cloned_roadmap = pldata

    for centre in colliding_cells:
            
        if centre in vertex_mapping:    

            for roadmap_vertex_1 in range(cloned_roadmap.numVertices()):

            
                if cloned_roadmap.getVertex(roadmap_vertex_1).getTag() in vertex_mapping[centre]:

                    cloned_roadmap.removeVertex(roadmap_vertex_1)
                    print(f"vertex {(roadmap_vertex_1)} removed!!")
                    roadmap_vertex_1-= 1


    for centre2 in colliding_cells:
            
        if centre2 in edge_mapping:
            
            for roadmap_vertex_2 in range(cloned_roadmap.numVertices()):

                for roadmap_vertex_3 in range(cloned_roadmap.numVertices()):

                    if cloned_roadmap.edgeExists(roadmap_vertex_2,roadmap_vertex_3):

                        if (cloned_roadmap.getVertex(roadmap_vertex_2).getTag(),cloned_roadmap.getVertex(roadmap_vertex_3).getTag()) in edge_mapping[centre2]:

                            cloned_roadmap.removeEdge(roadmap_vertex_2,roadmap_vertex_3)
                            cloned_roadmap.removeEdge(roadmap_vertex_3,roadmap_vertex_2)
                            print(f"edges in both directions -- {(roadmap_vertex_2,roadmap_vertex_3)} -- edge removed!!")

            # roadmap_vertex_1-= 1

    end_time = time.time()
    time_taken = end_time - start_time

    print(f"Number of Vertices before collision checking = {num_vertices}")
    print(f"Number of edges before collision checking = {num_edges} \n")

    print(f"Number of Vertices removed after collision checking = {num_vertices - cloned_roadmap.numVertices()}")
    print(f"Number of edges removed after collision checking = {num_edges - cloned_roadmap.numEdges()}")

    print(f"\nTime taken to remove vertices and edges = {time_taken} \n")

    
    # Save modified roadmap obtained after collision checking
    if robot == "fecth":    
        store_roadmap_vertices(cloned_roadmap,skel_tmp,context_tmp,request_tmp,world_tmp,"post_collision_roadmap.csv",robot)
    context_tmp.set_state(start_tmp(), world_tmp)


    
    if cloned_roadmap.numStartVertices() > 0:
        start_vertex_idx = cloned_roadmap.getStartIndex(0)
    else:
        raise ValueError("No start vertex found in the planner data.")

    if cloned_roadmap.numGoalVertices() > 0:
        goal_vertex_idx = cloned_roadmap.getGoalIndex(0)
    else:
        raise ValueError("No goal vertex found in the planner data.")
    
    reachable_data_1 = ob.PlannerData(context_tmp.si)  # ompl.base._base.PlannerData object
    
    cloned_roadmap.extractReachable(start_vertex_idx, reachable_data_1)

    print(f"Number of reachable vertices from start (after extractReachable): {reachable_data_1.numVertices()}")


    # # Check if goal vertex is in reachable_data
    goal_found_in_reachable = False
    goal_vertex = cloned_roadmap.getVertex(goal_vertex_idx)
    for i in range(reachable_data_1.numVertices()):
        if reachable_data_1.getVertex(i) == goal_vertex:
            goal_found_in_reachable = True
            break

    if goal_found_in_reachable:
        print("A path from start to goal exists in the updated roadmap (based on extractReachable).")
    else:
        print("No path exists from start to goal in the updated roadmap after adding obstacles.")



    if goal_found_in_reachable == True:
        # Construct a NetworkX graph for finding shortest path
        nx_graph = nx.DiGraph()

        # Add vertices to the NetworkX graph
        for i in range(cloned_roadmap.numVertices()):
            state = cloned_roadmap.getVertex(i).getState()
            nx_graph.add_node(i, state=state)
        
        # Add edges with weights to the NetworkX graph
        for i in range(cloned_roadmap.numVertices()):
            for j in range(cloned_roadmap.numVertices()):
                if cloned_roadmap.edgeExists(i, j):
                    edge_wt = ob.Cost()
                    cloned_roadmap.getEdgeWeight(i, j,edge_wt)  # Get the weight of the edge
                    edge_weight = edge_wt.value()
                    nx_graph.add_edge(i, j) #, weight=edge_weight
        print(f"number of edges in nx: {nx_graph.number_of_edges()}, number of vertices in nx: {nx_graph.number_of_nodes()}")
        path = nx.algorithms.shortest_paths.weighted.dijkstra_path(nx_graph, start_vertex_idx, goal_vertex_idx)
        
        print("Shortest path:", path)


        # Convert the path obtained from Djikstra's to a geometric path for visualization
        geometric_path = PathGeometric(context_tmp.si)
        # print(geometric_path)
        for node in path:

            graph_vertex_state = cloned_roadmap.getVertex(node).getState()
            geometric_path.append(graph_vertex_state)

        # geometric_path.interpolate()

        # Convert the waypoints into an executable trajectory.
        traj = path_to_trajectory(context_tmp, geometric_path)

        # Visualize path using PyBullet
        world_tmp.animate(traj, speedup = speedup)


    else:

        print("No path exists from start to goal after adding obstacles.")
        # Step 1 & 2: Extract reachable from start and goal
        reachable_data_2 = ob.PlannerData(context_tmp.si)
        context_tmp.extractReachable(goal_vertex_idx, reachable_data_2)
        print(f"Number of reachable vertices from goal (after extractReachable): {reachable_data_2.numVertices()}")


        # Convert reachable data to lists of states
        start_reachable_states = []
        for i in range(reachable_data_1.numVertices()):
            st = reachable_data_1.getVertex(i).getState()
            if st is not None:
                start_reachable_states.append(st)

        goal_reachable_states = []
        for i in range(reachable_data_2.numVertices()):
            st = reachable_data_2.getVertex(i).getState()
            if st is not None:
                goal_reachable_states.append(st)

        if not start_reachable_states or not goal_reachable_states:
            print("No states available in reachable sets. Cannot proceed with RRT connection.")
            return

        # Step 3: Pick random states and connect via RRT
        random_start_state = random.choice(start_reachable_states)
        random_goal_state = random.choice(goal_reachable_states)

        start_state_rrt = context_tmp.si.allocState()
        goal_state_rrt = context_tmp.si.allocState()

        context_tmp.si.getStateSpace().copyState(start_state_rrt, random_start_state)
        context_tmp.si.getStateSpace().copyState(goal_state_rrt, random_goal_state)

        rrt_context = get_OMPL_context(world_tmp, [request_tmp.group], "RRTConnect")

        start_state_array = rrt_context.state_to_array(start_state_rrt)
        scoped_start_state = rrt_context.scoped_state_from_array(start_state_array)

        goal_state_array = rrt_context.state_to_array(goal_state_rrt)
        scoped_goal_state = rrt_context.scoped_state_from_array(goal_state_array)

        rrt_context.clear_query()

        # Set start and goal state.
        rrt_context.setup.setStartAndGoalStates(scoped_start_state, scoped_goal_state)

        # Do motion planning.
        if rrt_context.plan(request_tmp.planning_time):
            print("RRT found a path connecting the two sets.")

            # Retrieve RRT path
            path_rrt = rrt_context.get_solution_path(simplify = simplify) # PathGeometric

            # Step 4 & 5: Find 3 paths
            # i) path from start to random_start_state
            start_path = shortest_path_in_roadmap(cloned_roadmap, context_tmp.si, start_tmp.get(), random_start_state)
            # ii) path from random_goal_state to goal
            goal_path = shortest_path_in_roadmap(cloned_roadmap, context_tmp.si, random_goal_state, goal_tmp.get())

            if start_path is None or goal_path is None:
                print("Could not find partial paths in the roadmap for start->random or random->goal.")
                return

            # Merge the three paths: start_path + path_rrt + goal_path
            final_path = PathGeometric(context_tmp.si)

            # Append start_path
            for i in range(start_path.getStateCount()):
                final_path.append(start_path.getState(i))
            
            # Append path_rrt except its first state (to avoid duplication)
            for i in range(1, path_rrt.getStateCount()):
                final_path.append(path_rrt.getState(i))

            # Append goal_path except its first state (to avoid duplication)
            for i in range(1, goal_path.getStateCount()):
                final_path.append(goal_path.getState(i))

            # Visualize final path
            traj_final = path_to_trajectory(rrt_context, final_path)
            world_tmp.animate(traj_final, speedup=speedup)

        else:
            print("RRT could not find a path to connect the two reachable sets.")







if __name__ == '__main__':
    Fire(main)
