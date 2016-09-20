function h = drawcar(param)
%--------------------------------------------------------------------------
% H = DRAWRECTANGLE(PARAM,STYLE)
% This function draws a rectangle with the given parameters:
% - inputs:
%          param................... 1x5 array
%          - param = [a, b, w, h, theta]
%          - (a,b): the center of the rectangle
%          - (w,h): width and height of the rectangle > 0
%          - theta: the rotation angle of the rectangle
%          style................... string
%          - plot style string
% - output:
%          h....................... plot handler
%
%   Usage Examples,
%
%   DrawRectangle([0 0 1 1 0]); 
%   DrawRectangle([-1,2,3,5,3.1415/6],'r-');
%   h = DrawRectangle([0,1,2,4,3.1415/3],'--');
%
%   Rasoul Mojtahedzadeh (mojtahedzadeh _a_ gmail com)
%   Version 1.00
%   November, 2011
%--------------------------------------------------------------------------

% if (nargin <1),
%     error('Please see help for INPUT DATA.');
% elseif (nargin==1)
%     style='r-';
% end;
% [m,n] = size(param);
% if(m ~= 1 || n ~= 5)
%     error('param should be an 1x5 array.');
% end
% if(param(3) <=0 || param(4) <=0)
%     error('width and height must be positive values.');
% end
a = param(1);
b = param(2);
w = 4;
h = 2;
theta = param(3);
X = [-w/2 w/2 w/2 -w/2 -w/2];
Y = [h/2 h/2 -h/2 -h/2 h/2];
P = [X;Y];
ct = cos(theta);
st = sin(theta);
R = [ct -st;st ct];
P = R * P;

r = 0.7; % magnitude (length) of arrow to plot
u = r * cos(theta); % convert polar (theta,r) to cartesian
v = r * sin(theta);
quiver(a,b,u,v, 'MaxHeadSize', 2);
hold on;
h=plot(P(1,:)+a,P(2,:)+b);
axis equal;
end