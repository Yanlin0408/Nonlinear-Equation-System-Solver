function [J] = fdJacob2D(l, theta, alpha)
    J = [0,0;0,0];
    % first param of Jacobian
    
    [pos1,Jacob1] = evalRobot2D(l,theta+[alpha;0]);
    [pos2,Jacob2] = evalRobot2D(l,theta-[alpha;0]);
    J_first = (pos1-pos2)/(2*alpha);
    
    % second param of Jacobian
    [pos3,Jacob3] = evalRobot2D(l,theta+[0;alpha]);
    [pos4,Jacob4] = evalRobot2D(l,theta-[0;alpha]);
    J_sec = (pos3-pos4)/(2*alpha);
    
    J(:,1) = J_first;
    J(:,2) = J_sec;
    
