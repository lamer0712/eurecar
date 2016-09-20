function out = parking_path(park_point, state)
% close all
% align final heading
% state and park_point
r_min = 3.8;
x_1 = [];
xynew_1 = [];
xynew_2 = [];
xynew_3 = [];
angle = 45*pi/180;
angle_1 = 1*pi/180;

x2 = park_point(1);
y2 = park_point(2);
theta2 = park_point(3);
theta2 = wrap(theta2);

x1 = state(1);
y1 = state(2);
theta1 = state(3);
theta1 = wrap(theta1);

del_theta = theta2 - theta1;

theta = atan2((y2-y1),(x2-x1));

% state
if (theta1>theta); %RC
    c_x1 = x1+r_min*cos(pi/2-theta1);
    c_y1 = y1-r_min*sin(pi/2-theta1);
    theta_s = pi/2 + theta1 - angle;
    theta_s1 = -pi/2;
%     circle(c_x1,c_y1,r_min);
%     hold on
elseif (theta1<theta); %LC
%     c_x1 = x1-r_min*cos(pi/2-theta1);
%     c_y1 = y1+r_min*sin(pi/2-theta1);
%     theta_s = -pi/2 + theta1 + angle;
%     theta_s1 = pi/2;
%     circle(c_x1,c_y1,r_min);
%     hold on

    c_x1 = x1+r_min*cos(pi/2-theta1);
    c_y1 = y1-r_min*sin(pi/2-theta1);
    theta_s = pi/2 + theta1 - angle;
    theta_s1 = -pi/2;
%     circle(c_x1,c_y1,r_min);
%     hold on
end
% parking_point
if (theta2>theta); %LC
    c_x2 = x2-r_min*cos(pi/2-theta2);
    c_y2 = y2+r_min*sin(pi/2-theta2);
    theta_p = -pi/2 + theta2 - angle_1;
    theta_p1 = pi/2;
%     circle(c_x2,c_y2,r_min);
%     hold on
elseif (theta2<theta); %RC
    c_x2 = x2+r_min*cos(pi/2-theta2);
    c_y2 = y2-r_min*sin(pi/2-theta2);
    theta_p = pi/2 + theta2 + angle_1;
    theta_p1 = -pi/2;
%     circle(c_x2,c_y2,r_min);
%     hold on
end

xy = [cos(theta1) sin(theta1) ; -sin(theta1) cos(theta1)] * [x2 - x1 ; y2 - y1];

x = xy(1);
y = xy(2);

A = [x^3 x^2 ; 3*x^2 2*x]^-1 * [y ; tan(del_theta)];

xnew = [0 : 0.1 : x];
ynew = A(1) * xnew.^3 + A(2) * xnew.^2;
xy_1 = [cos(theta1) -sin(theta1) ; sin(theta1) cos(theta1)] * [xnew ; ynew];

xnew = xy_1(1,:) + x1;
ynew = xy_1(2,:) + y1;
xynew = [xnew' ynew'];

rp_data = get_radius(xnew,ynew);

% % % % % % % plot(xnew,ynew,'*b');
% % % % % % % hold on;

% drawcar([x1 y1 theta1]);
% drawcar([x2 y2 theta2]);

if(size(rp_data,1) ~= 0)
% % % % % % %     %     x_new = rp_data(1,:)+x1
% % % % % % %     %     y_new = rp_data(2,:)+y1
    x_new = rp_data(1,:);
    y_new = rp_data(2,:);
    
    if (size(x_new,2)>0);
        for i = 1 : size(x_new,2);
            d(i) = sqrt((x1 - x_new(i))^2 + (y1 - y_new(i))^2);
        end
        % state
        if(min(d)<r_min);
            [x, y] = pol2cart(theta_s,r_min);
%             plot(x+c_x1,y+c_y1,'*k');
            x_1 = x+c_x1;
            y_1 = y+c_y1;
            theta_1 = theta_s + theta_s1;
            state = [x1 y1 theta1];
            park_point = [x_1 y_1 theta_1];
            xynew_1 = parking_path_1(park_point, state);
        elseif(min(d)>=r_min)
            x_1 = x1;
            y_1 = y1;
            theta_1 = theta1;
        end
        % parking_point
        for i = 1 : size(x_new,2);
            d_1(i) = sqrt((x2 - x_new(i))^2 + (y2 - y_new(i))^2);
        end
        if(min(d_1)<0.01);
            [x, y] = pol2cart(theta_p,r_min);
%             plot(x+c_x2,y+c_y2,'*k');
            x_2 = x+c_x2;
            y_2 = y+c_y2;
            theta_2 = theta_p + theta_p1;
            park_point = [x2 y2 theta2];
            state = [x_2 y_2 theta_2];
            xynew_2 = parking_path_1(park_point, state);
        elseif(min(d_1)>=0.01)
            x_2 = x2;
            y_2 = y2;
            theta_2 = theta2;
        end
        
        if(min(d_1)>=r_min && min(d)>=r_min)
            return
        end
        state = [x_1 y_1 theta_1];
        park_point = [x_2 y_2 theta_2];
        xynew_3 =parking_path_1(park_point,state);
        xynew = [xynew_1 ; xynew_2 ; xynew_3];
    end
elseif(size(rp_data,1) == 0)
%     plot(xnew,ynew,'*r');
%     hold on;
end

out = xynew;