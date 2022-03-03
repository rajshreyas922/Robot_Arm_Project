plotRobot3D([1,1,1]',[pi/4, pi/2, pi]') %Elbow at pi/2
plotRobot3D([1,1,1]',[pi, 2*pi, pi]') %Fully stretched arm
plotRobot3D([1,1,1]',[pi/2, pi, pi]') %arm at centre


mode = 0
l = [0.8 0.7 1];
start = rand(3,1);
start = [pi 2*pi pi]' %Does not work with simple method
plotRobot3D(l,start);
[pos, ~]=evalRobot3D(l, start) %[0.9031 0.5214 1.0062]
 
%b.
desired = -pos
t = invKin3D(l, start, desired, 200, 0);
plotRobot3D(l,t);
d = sqrt(sum((desired - [1.5000   0    0.0000]).^2))

%The Newton's method accuracy decreases as the target point gets too far
%away from the end effector's original position. This is because when J is
%large values, the condition number also increases a lot, as a result,
%theta's updates are suceptible to large changed even with small errors.
%The closer the targer is to the end effector, the more accurate results we
%get. The convergence also depends on the starting point.

n = 20
start_d = evalRobot3D(l, start)
%Draw a line joining the start and end points.
A = linspace(start_d(1), desired(1),n);
B = linspace(start_d(2), desired(2),n);
C = linspace(start_d(3), desired(3),n);

%Breaking the path into two segments.
C(int16(n/2)) = C(int16(n/2))+0.5;
s1_x = linspace(start_d(1), A(int16(n/2)), n);
s1_y = linspace(start_d(2), B(int16(n/2)), n);
s1_z = linspace(start_d(3), C(int16(n/2)), n);

s2_x = linspace(A(int16(n/2)), desired(1), n);
s2_y = linspace(B(int16(n/2)), desired(2), n);
s2_z = linspace(C(int16(n/2)), desired(3), n);
plotRobot3D(l,start);
t = start

%Slowly move towards the end points by solving for theta in consecutive
%steps instead of all at once.
for i = [1:20]
    t = invKin3D(l, t, [s1_x(i) s1_y(i) s1_z(i)], 100, mode);
    plotRobot3D(l,t);
end

for i = [1:20]
    t = invKin3D(l, t, [s2_x(i) s2_y(i) s2_z(i)], 100, mode);
    plotRobot3D(l,t);
end

plotRobot3D(l,t);