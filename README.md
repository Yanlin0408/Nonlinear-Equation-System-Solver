# Nonlinear-Equation-System-Solver

Background
Given a kinematic chain (a set of joints that are joined by links), forward kinematics is the problem of determining the position of the joints (or only the end-effector) given the joint angles. This problem is a straighforward application of Euclidean transformations.
Inverse kinematics is the inverse problem: determine the joint orientations that satisfy a given a set of constraints (e.g., find the joint orientations that put the end-effector at a specific location). This problem is harder than forward kinematics; the analytic solution is often complicated or doesn't exist so numerical solutions are required. Furthermore, it is possible that the solution is not unique. There may be no solutions or several solutions.
