function [theta] = invKin2D(l,theta0,pos,n,mode)
%EVALKIN2D Summary of this function goes here
%   Detailed explanation goes here
theta_i = theta0
if mode == 0
    for i = [1:n]
        [pos_t, A] = evalRobot2D(l, theta_i);
        J = fdJacob2D(l, theta_i, 0.0001);
        s_i = -J\(pos_t - pos);
        theta_i = theta_i + s_i;
        if (abs(norm(pos_t - pos))) < 0.001
            break;
        end
    end
    theta = theta_i;
end

if mode == 1
    [pos_t,B] = evalRobot2D(l,theta0);
    for i=1:n
        s_i = B\-(pos_t-pos)
        theta_new = theta_i + s_i;
        [pos_t_new, N] = evalRobot2D(l,theta_new);
        y = pos_t_new - pos_t;
        B = B + ((y - B*s_i)*s_i')/(s_i'*s_i);
        theta_i = theta_new;
        pos_t = pos_t_new;
        if (abs(norm(pos_t - pos))) < 0.001
            break;
        end
    end
    theta = theta_i;
end




end

