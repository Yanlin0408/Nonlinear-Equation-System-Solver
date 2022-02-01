function [theta]=invKin3D(l,theta0,desired,n,mode)
%invKin3D([1;1],[pi/4;pi/8;pi/4],[7.49879891330929e-33;1.22464679914735e-16;2],5,0)
    if mode == 0
        theta_now = theta0;
        [initial_pos,initial_J] = evalRobot3D(l,theta_now);
        if cond(initial_J) == Inf
            theta_now = theta_now + 0.2;
            initial_pos = initial_pos + 0.1;
        end
        num_segs = 40;
        segs_x = linspace(initial_pos(1,1),desired(1,1),num_segs);
        segs_y = linspace(initial_pos(1,2),desired(2,1),num_segs);
        segs_z = linspace(initial_pos(1,3),desired(3,1),num_segs);
        segs(1,:) = segs_x';
        segs(2,:) = segs_y';
        segs(3,:) = segs_z';
        segs = segs';
        disp(segs');
        for cc = 2:length(segs)
            for i=1:n
                % move to segs(cc,:)
                desired_now = segs(cc,:)';
                [updated_pos, Jacob] = evalRobot3D(l,theta_now);
                f = updated_pos' - desired_now;
                theta_now = theta_now - Jacob\f;
            end
        end
%         for i=1:n
%             [position, Jacob] = evalRobot3D(l,theta_now);
%             f = position' - desired;
%             %theta_now = theta_now - inv(Jacob)*f;
%             theta_now = theta_now - Jacob\f;
%         end   
    elseif(mode == 1)
        % For Broyden's method start with the Jacobian from your evalRobot2D function.
        theta_now = theta0;
        [position, Jacob] = evalRobot3D(l,theta_now);
        B_now = Jacob;
        for i=1:n
           [position_1, Jacob] = evalRobot3D(l,theta_now);
           f_now = position_1' - desired;
           s_now = -B_now\f_now ;
           theta_now = theta_now + s_now;
           
           [position_2, Jacob] = evalRobot3D(l,theta_now);  
           f_kplus1 = position_2' - desired; % f(x_k+1)
           y_now = f_kplus1 - f_now ;
           B_now = B_now + ((y_now - (B_now*s_now))*s_now')/(s_now'*s_now);
           % plotRobot2D(l,theta_now);
           % pause();
        end
    end
    
    theta = theta_now;
    