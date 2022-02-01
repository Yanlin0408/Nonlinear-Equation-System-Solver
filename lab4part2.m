% question 1: Use the above method to display the Jacobian for the following configurations and explain what happens:

% a. Elbow at pi/2
    % ans:
    % when any one of the elbows is at pi/2, one link would be
    % perpendicular to the other one.
    % And the condition number of corresponding Jacobian would be Inf,
    % which means that the condition is huge and Jacobian is singular and
    % the problem is ill-conditioned.
    % When the robot encounters these kind of positions during inverse
    % Kinematics (using Newton's), starting close to 
    % the root doesn't help, because small residual implies accurate
    % solution when problem is well-conditioned.
    
% b. Arm fully stretched in a line
    % ans:
    % when angels = [pi/2,0,pi/2], the Arm is fully stretch in a line
    % the corresponding Jacob we got from evalRobot3D is 
    % J = [-1.22464679914735e-16,-6.12323399573677e-17,-1.22464679914735e-16;
    %      -2,-1,7.49879891330929e-33;
    %      1.22464679914735e-16,6.12323399573677e-17,0]
    % The rank of this Jacobian is 1.
    % The Jacobian is singular, which means this is ill-conditioned.
    
% c. Arm at the center
    % ans:
    % When angels = [-pi/2,-pi,-pi/2], the arm is almost folded and is
    % close to the center of the robot.
    % The condition number of the Jacobian is 8.1656e+15, which implies
    % that this is ill-conditioned and the Jacobian is almost singular.
    % When the robot encounters these kind of positions during inverse
    % Kinematics (using Newton's), starting close to 
    % the root doesn't help, because small residual implies accurate
    % solution when problem is well-conditioned.

% question 2
%[7.49879891330929e-33;1.22464679914735e-16;2] is the result we get from evalRobot3D([1;1],[pi/2;0;pi/2])
[theta_Q2_1] = invKin3D([1;1],[pi/4;pi/9;pi/2],[7.49879891330929e-33;1.22464679914735e-16;2],25,0);
[theta_Q2_2] = invKin3D([1;1],[pi/4;pi/9;pi/2],[-7.49879891330929e-33;-1.22464679914735e-16;-2],25,1);

% a. Start moving the target point farther from the end effector and check where the Newton’s method stops converging. Explain why it stops converging.
    % ans:
    % When the difference of angles is considerable, we receive the warning
    % of "Matrix close to singular"
    % Because the convergence rate of Newton’s method for simple root is quadratic
    % , r = 2. We must start close enough to root to converge.
    
% b. How would you tackle the problem above where the target is far from the end effector?
    % ans: we should use spline or linspace to slice up the trajectory
    % between out initial guess and our desired point. My code perfectly
    % handles that.
    
% c. Choose target points in the diametrically opposite quadrant to the end effector and try to reach the point using your algorithm above. What could be the issue here? Plan a path to the target and show how that would solve this issue.
    % ans: 
    % The issue is: While one coordinate in the 3*1 position matrix stays
    % completely the same, two other coordinates changed significantly when
    % every angles are supposed to be completely opposite. Like
    % [pi/2,0,pi/2] should become [-pi/2,0,-pi/2], which is how we get the
    % diametrically opposite end effector.
    % Plan to solve this issue:
    % We use divide the path from start point to end point into separate
    % segments (10 to 20) by spline(). After spliting the path into segments, we can
    % iterate through each segments in the vector returned by spline() and
    % don't need to worry about issue of far target from the end effector.
    
% question 3
