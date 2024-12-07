
function [theta1, theta2, Lb] = get_thetas_xy(x,y,params)
    L1 = params.L1;
    r = params.r;
    theta1 = asin((y-r)/L1);
    theta2 = pi - theta1;
    Lb = x-L1*cos(theta1);
end