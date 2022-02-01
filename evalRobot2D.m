function [pos, J] = evalRobot2D(l, theta)
    l1 = l(1,1);
    l2 = l(2,1);
    T1 = theta(1,1);
    T2 = theta(2,1);
    pos = [0;0];
    pos(1,1) = l1*cos(T1) + l2*cos(T1+T2);
    pos(2,1) = l1*sin(T1) + l2*sin(T1+T2);
    
    J = [0,0;0,0];
    J(1,1) = -l1*sin(T1) - l2*sin(T1+T2) ;
    J(2,1) = l1*cos(T1) + l2*cos(T1+T2) ;
    J(1,2) = -l2*sin(T1+T2) ;
    J(2,2) = l2*cos(T1+T2) ;
