function [theta] =invKin2D(l,theta0,pos,n,mode)
    if mode == 0
        theta_now = theta0;
        for i=1:n
            [position, Jacob] = evalRobot2D(l,theta_now);
            f = position - pos;
            %theta_now = theta_now - inv(Jacob)*f;
            theta_now = theta_now - Jacob\f;
        end   
    elseif(mode == 1)
        % For Broyden's method start with the Jacobian from your evalRobot2D function.
        theta_now = theta0;
        [position, Jacob] = evalRobot2D(l,theta_now);
        B_now = Jacob;
        for i=1:n
           [position_1, Jacob] = evalRobot2D(l,theta_now);
           f_now = position_1 - pos;
           %s_now = -inv(B_now)*f_now ;
           s_now = -B_now\f_now ;
           theta_now = theta_now + s_now;
           [position_2, Jacob] = evalRobot2D(l,theta_now);  
           f_kplus1 = position_2 - pos; % f(x_k+1)
           y_now = f_kplus1 - f_now ;
           B_now = B_now + ((y_now - (B_now*s_now))*s_now')/(s_now'*s_now);
           % plotRobot2D(l,theta_now);
           % pause();
        end
    end
    
    theta = theta_now;