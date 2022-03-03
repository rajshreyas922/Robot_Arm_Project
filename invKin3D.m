function theta=invKin3D(l,theta0,desired,n,mode)
%Returns the angles required to reach desired position
theta_i = theta0;
if mode == 0
    for i = [1:n]
        [pos_t, J] = evalRobot3D(l, theta_i);
        s_i = -J\(pos_t - desired)';
        theta_i = theta_i + s_i;
        if (abs(norm(pos_t - desired))) < 0.001
            break;
        end
    end
    theta = theta_i(:,1);
end

if mode == 1
    [pos_t,B] = evalRobot3D(l,theta_i)
    for i=1:n
        s_i = -B\(pos_t-desired)';
        theta_new = theta_i + s_i;
        [pos_t_new, N] = evalRobot3D(l,theta_new);
        y = (pos_t_new - pos_t)'
        B = B + ((y - B*s_i)*s_i')/(s_i'*s_i);
        theta_i = theta_new
        pos_t = pos_t_new;
        if (abs(norm(pos_t - desired))) < 0.001
            break;
        end
    end
    theta = theta_i;
end

end

