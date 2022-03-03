function [J] = fdJacob2D(l,theta, alpha)
%UNTITLED Return the approximate value of the jacboian using central
%differences0
%Are the results close enough to be useful in the optimization?
%Answer: The results are close enough for practical use

%Why would you use this finite-difference approximation instead of the
%analytic derivative?
%
%Finite differencec approximation is easier to compute compared to the
%analytical difference.

 J(:,1) = (evalRobot2D(l,theta+[alpha;0]) - evalRobot2D(l,theta-[alpha;0]))/(2*alpha);
 J(:,2) = (evalRobot2D(l,theta+[0;alpha]) - evalRobot2D(l,theta-[0;alpha]))/(2*alpha);
end
