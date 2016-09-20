ccc

load('partilane2.mat');
load('model.mat');

n = 100;
m = size(model,1);
mu = 0;%int32(n*0.02);

xmin = min(lane(:,1));
xmax = max(lane(:,1));
ymin = max(3,min(lane(:,2)));
ymax = max(lane(:,2));

lamdaxy = 0.225/2;
lamdate = 3/180*pi;
rangete = 50/180*pi;

rx = (xmax-xmin)*rand(n,1)+xmin;
ry = (ymax-ymin)*rand(n,1)+ymin;
rt = 2*rangete*rand(n,1)-rangete;
nn = 0;

figure_on = 1;
%%
tic
for k = 1:100
    
    if(figure_on)
        figure(1)
        hold off
        plot(rx,ry,'k.')
        hold on
    end
    weight =  zeros(size(rx,1),1);
    for i = 1:size(rx,1)
        
        rotM = [cos(rt(i)) -sin(rt(i)); sin(rt(i)) cos(rt(i))];
        model_sub = (rotM*model')';
        model_sub(:,1) = model_sub(:,1) + rx(i);
        model_sub(:,2) = model_sub(:,2) + ry(i);
        
        if(0)%sum((rx(i)-obs(:,1)).^2+(ry(i)-obs(:,2)).^2<(lamdaxy*4)^2)>0)
            weight_sub = 0.0;
        else
            weight_sub = 1.0;
            for j = 1:m
                weight_temp = (-1/0.225).*(sqrt(min((model_sub(j,1)-lane(:,1)).^2+(model_sub(j,2)-lane(:,2)).^2)))+1;
                weight_temp = sum((weight_temp>0&weight_temp<1).*weight_temp+weight_temp>=1);
%                 weight_temp = (weight_temp>=1)*weight_temp + (weight_temp<1)*0.9;
                weight_sub = weight_sub + weight_temp;
            end
        end
        weight_sub
        weight(i,1) = weight_sub;
        
        if(figure_on)
            if(weight_sub>0.0)
                plot(model_sub(:,1), model_sub(:,2),'c')
            else
                plot(model_sub(:,1), model_sub(:,2),'y')
            end
        end
        
    end
    
    weight = weight./sum(weight);
    intweight = int32(n*weight);
    intweight = int32(n^2/sum(intweight)*weight);
    
    rx_new = [];
    ry_new = [];
    rt_new = [];
    
    for i = 1 : size(rx,1)
        rx_new = [rx_new ; rx(i) + (rand(intweight(i,1),1).*2-1).*lamdaxy];
        ry_new = [ry_new ; ry(i) + (rand(intweight(i,1),1).*2-1).*lamdaxy];
        rt_new = [rt_new ; rt(i) + (rand(intweight(i,1),1).*2-1).*lamdate];
    end
    
    if(mu>0)
        nn = n+mu-size(rt_new,1);
        if(nn>0)
            rx_new = [rx_new ; (xmax-xmin)*rand(nn,1)+xmin];
            ry_new = [ry_new ; (ymax-ymin)*rand(nn,1)+ymin];
            rt_new = [rt_new ; 2*lamdate*4*rand(nn,1)-lamdate*4];
        else
            nn = 0;
        end
    end
    
    rx = rx_new;
    ry = ry_new;
    rt = rt_new;
    
    if(figure_on)
        plot(lane(:,1),lane(:,2),'r.')
        plot(obs(:,1),obs(:,2),'m.')
        axis equal
        figure(2)
        subplot(2,1,1)
        plot(weight)
        subplot(2,1,2)
        plot(intweight,'.')
        [k sum(intweight) nn]
%         pause
    end
end
toc












