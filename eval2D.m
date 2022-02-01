%This code is available in eval2D.m
% make sure you define n and mode
ls=[0.5,0.5]';
t=rand(2,1); %Choose some random starting point.

clf;
plotRobot2D(ls,t);
hold off;

while(1)
  desired=ginput(1)'; %Get desired position from user

  clf;
  plot(desired(1),desired(2),'*');
  hold on;
  plotRobot2D(ls,t,':');
  
  %Solve and display the position
  
  % free to switch mode. Effect is pretty much the same
  mode = 0;
  t=invKin2D(ls,t,desired,10,mode); 
  plotRobot2D(ls,t);
  hold off;
end

