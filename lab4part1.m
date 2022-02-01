% question 1
l = [0.5;0.5];
theta = [pi/4;pi/4];
[position_Q1,Jacob_Q1] = evalRobot2D(l,theta);

% question 2
Jacob_Q2 = fdJacob2D(l,theta,0.1);
% Are the results close enough to be useful in the optimization
% ans: yes
% Why would you use this finite-difference approximation instead of the analytic derivative?
% ans: two cases: 1, when calculating analytic derivative is too expensive.
%                 2, when we are only given datasets, not the function, we
%                 can only approximate the partial derivatives to form the Jacobian. 

% question 3
% Newton
n = 10;
[theta_Q3_1] =invKin2D(l,[2;2],position_Q1,n,0);
% Broyden's
n = 10;
[theta_Q3_2] =invKin2D(l,[2;2],position_Q1,n,1);

% question 4

% NOTE: ran the script inside eval2D.m, which I also submitted

% how does Broyden's compare to Newton?
% ans: Broyden's is not as accurate and stable as Newton. since in the
%      correction part of B ((y-B*s)*s'/(s'*s)), s'*s is used as a denominator,
%      where s is the amount of change of the angles (change of theta),
%      which means the result we get is affected by s directly.
%      So when we do many iterations and s gets tiny, the correction of B matrix
%      would be crazily big. Or when the change of theta is too big (difference of
%      the previous position to position of destination), we would not be 
%      able to reach our destination ideally, since the amount of correction, B, is 
%      relatively mininum comparing to what Jacobian supposed to be.
%      That's why we need to use a threshold on the norm of the residual 
%      as a constraint. This is the drawback of Broyden's. Whereas we
%      always get ideal output using Newton's as long as we do enough
%      iterations.

%      The advantage of
%      Broyden's is that we can approximate the Jacobian by continuously giving
%      it correction instead of explicitly computing it. And we can use the
%      "Jacobian" that we approximated, which is B, and the change in position
%      of the end effector to compute the change in angels, which is s, and
%      append it to the x we got in previous iteration.