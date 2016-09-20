function out=wrap(angle)

% angle in radian

if (angle < -pi) 	%% correct delta yaw rollover
    angle=angle+2.*pi;
    
elseif (angle > pi)
        angle=angle-2.*pi;
end
out=angle; % return in radian