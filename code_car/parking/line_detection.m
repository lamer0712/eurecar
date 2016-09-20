close all
pixel_size = 0.1;

xmin = min(lanex);
xmax = max(lanex);
ymin = min(laney);
ymax = max(laney);

lane = (zeros(ceil((xmax-xmin)/pixel_size),ceil((ymax-ymin)/pixel_size)))';
lane_size = size(lane);
for i = 1 : length(lanex)
    indy(i) = int32((lanex(i)-xmin)/pixel_size);
    indx(i) = int32((ymax-laney(i))/pixel_size);
    if(indx(i)>0&&indy(i)>0&&indx(i)<=lane_size(1)&&indy(i)<=lane_size(2))
        lane(indx(i),indy(i)) = 1;        
    end
end
% H = [];
% T = [];
% R = [];
% P = [];
% 
% 
% for i = 1 : 18
%     [H_sub,T_sub,R] = hough(lane,'Theta',(i-10)*10:0.1:(i-9)*10-0.1);
%     P_sub  = houghpeaks(H_sub,100,'threshold',ceil(0.1*max(H_sub(:))));
%     P_sub(:,2) = P_sub(:,2) + size(T,2);
%     H = [H H_sub];
%     T = [T T_sub];    
%     P = [P; P_sub];    
% end    
% lane = edge(lane,'canny',0.1);
%%
[H,T,R] = hough(lane,'Theta',-90:0.1:90-0.1);%(-68:0.1:-65)+90-30+90
P  = houghpeaks(H,1000,'threshold',ceil(0.1*max(H(:))));
lines = houghlines(lane,T,R,P,'FillGap',5,'MinLength',2/pixel_size);
length(lines)

figure, hold on
spy(lane);
thetas = [];
rhos = [];
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   thetas = [thetas;lines(k).theta];
   rhos = [rhos;lines(k).rho];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'m.');
   plot(xy(2,1),xy(2,2),'r.');
   lane(xy(2,2),xy(2,1)) = 0;
   lane(xy(1,2),xy(1,1)) = 0;
end

figure
imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
axis on, axis normal, hold on;
plot(T(P(:,2)),R(P(:,1)),'s','color','white');

figure
hist(thetas,18)

figure
hist(rhos)
