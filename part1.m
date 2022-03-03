%This code is available in eval2D.m
% make sure you define n and mode
ls=[1,1]';
t=rand(2,1); %Choose some random starting point.
n = 10
mode = 1
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
  t=invKin2D(ls,t,desired,n,mode) 
  plotRobot2D(ls,t);
  hold off;
end
%How does Broyden's compare to Newton?
%Both of them has similar results but sometimes, broyden's method ends up
%with singularity problems.