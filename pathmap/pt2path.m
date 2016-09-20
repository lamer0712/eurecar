 function [path, r, v] = pt2path(subpoint,evals,ap,an,m_speed,mu,l_speed)
%subpoint = wpt;evals = 0.15;ap = 2.77778;an = -1.73611;m_speed = 200;mu = 0.186805;
% close all
out_temp = [];
way = subpoint;
sub = subpoint(1,:);
evals = 1

for i = 2 : size(subpoint,1)
    if((sum((subpoint(i,1:2)-sub(end,1:2)).^2))>=0.25)
        sub(end+1,:) = subpoint(i,:);
    end 
end

subpoint = sub;

idx = size(subpoint,1);

while(idx>=3)
    min_idx = size(subpoint,1);
    val = inf;
    h01 = atan2(subpoint(2,2)-subpoint(1,2),subpoint(2,1)-subpoint(1,1));
    h02 = atan2(subpoint(3,2)-subpoint(2,2),subpoint(3,1)-subpoint(2,1));
    
    h0 = mod(h01-h02,2*pi);
    h0 = h0 + (h0>pi)*(-2*pi) + (h0<-pi)*2*pi;
    h0 = h0*0.5+h02;
    h0 = h0 + (h0>pi)*(-2*pi) + (h0<-pi)*2*pi;

    temp = [subpoint(:,1)-subpoint(1,1) subpoint(:,2)-subpoint(1,2)];
    T = [cos(h0) sin(h0); -sin(h0) cos(h0)];
    temp = (T*[temp(:,1)';temp(:,2)'])';
    
    while(val>evals)
        idx = min_idx;
        t1 = (atan2(temp(idx,2)-temp(idx-1,2),temp(idx,1)-temp(idx-1,1)));
        t2 = (atan2(temp(idx-1,2)-temp(idx-2,2),temp(idx-1,1)-temp(idx-2,1)));
        t = mod(t1-t2,2*pi);
        t = t + (t>pi)*(-2*pi) + (t<-pi)*2*pi;
        t = (t*0.5+t2);
        t = t + (t>pi)*(-2*pi) + (t<-pi)*2*pi;
        t = tan(t);

        x = temp(idx,1);
        y = temp(idx,2);
        
        a = (x*t-2*y)/x^3;
        b = (3*y-x*t)/x^2;
        [val,min_idx] = max(abs(a*temp(1:idx,1).^3+b*temp(1:idx,1).^2-temp(1:idx,2)));
        if(isnan(val))
            val = inf;
            min_idx = idx-1;
        elseif(val>evals)
            min_idx = ceil(min_idx*0.5+idx*0.5);            
        end
    end
    
    if(idx>=3)         
        out_temp = [out_temp; subpoint(1,1) subpoint(1,2) h0 x a b 0 0];
        subpoint = subpoint(idx:end,:);
        idx = size(subpoint,1);
    end
end

r = [inf 0 0];
for i = 1:size(out_temp,1)
    x = [0:0.1:out_temp(i,4)];
    if(x(end)~=out_temp(i,4))
        x(end+1) = out_temp(i,4);
    end
    a = out_temp(i,5);
    b = out_temp(i,6);
    y = a*x.^3+b*x.^2;
    yp = 3*a*x.^2+2*b*x;
    ypp = 6*a*x+2*b;
    r_temp = ((1+yp.^2).^1.5)./ypp;
    d = zeros(size(x,2),1);
    for j = 2: size(x,2)
        d(j) = sqrt((x(j)-x(j-1)).^2+(y(j)-y(j-1)).^2);
    end
    
    r = [r; r_temp(2:end)' zeros(size(x,2)-1,1)+i d(2:end)];
end

r(1,4) = 0;
for i = 2 : size(r,1)
    r(i,4) = r(i-1,4)+r(i,3);
end

r(end,1) = (l_speed/3.6)^2/9.81/mu;


v2 = min(m_speed/3.6,sqrt(mu*9.81*abs(r(:,1))));
v = zeros(size(r,1),1)+m_speed;

while(sum(v>v2))
    [val,idx] = min((v>v2).*v2+(v<=v2).*m_speed);
    d = r(:,4)-r(idx,4);
    v = min(min(v,sqrt(2*(d>0).*ap.*d+2*(d<=0).*an.*d+val^2)),m_speed/3.6);
end

idx = 2;
for i = 1:size(out_temp,1)
    v_temp = v(idx:idx+sum(r(:,2)==i)-1);
    v2_temp = v2(idx:idx+sum(r(:,2)==i)-1);
    
    x = [0.1:0.1:out_temp(i,4)];
    if(x(end)~=out_temp(i,4))
        x(end+1) = out_temp(i,4);
    end
    
    [chk,min_idx] = max((v2_temp==v_temp));
    if(chk)
        if(min_idx==1)
            [~,min_idx] = max((v2_temp~=v_temp));
        end
    else
        [~,min_idx] = max(v_temp);
    end    
    out_temp(i,7) = min(v_temp)*3.6;
    out_temp(i,10:13) = [v_temp(1)*3.6  v_temp(min_idx)*3.6 v_temp(end)*3.6 x(min_idx)];
    
    idx = idx+sum(r(:,2)==i);    
end

for i = 1:size(out_temp,1)
    a = out_temp(i,5);
    b = out_temp(i,6);
    xe= out_temp(i,4);
    dx = 0:0.01:xe;
    if(dx(end)~=xe)
        dx(end+1) = xe;
    end
    
    dy = (3*a*dx.^2+2*b*dx)*0.01;
    for j = length(dx):-1:2
        dx(j) = dx(j)-dx(j-1);
    end
    
    out_temp(i,9) = sum(sqrt(dx.^2+dy.^2));
end

path = out_temp';

figure(1)
hold on
plot(v*3.6,'b')

figure(2)
hold on
plot(way(:,1),way(:,2),'.')
for path_idx = 1: size(path,2)    
    sub_x = [0: 0.5 : path(4,path_idx), path(4,path_idx)];
    sub_y = path(5,path_idx)*sub_x.^3+path(6,path_idx)*sub_x.^2;
    sub_temp = [sub_x;sub_y];
    Rot = [cos(path(3,path_idx)) sin(path(3,path_idx)) ; -sin(path(3,path_idx)) cos(path(3,path_idx))];
    xy = Rot'*sub_temp;
    load_x = xy(1,:)+ path(1,path_idx);
    load_y = xy(2,:)+ path(2,path_idx);
    load_t = path(8,path_idx)+0*load_y;
    color = [rand rand rand];
    plot(load_x,load_y,'r')  
    
end
plot(out_temp(:,1),out_temp(:,2),'r*')
axis equal
    




