ccc

load('partilane5.mat');
load('model2.mat');

n = 100;
m = size(model,1);
np = int32(n*0.20);
max_iter = 10;

xmin = min(lane(:,1));
xmax = max(lane(:,1));
ymin = min(lane(:,2));
ymax = max(lane(:,2));

lamdaxy = 0.225/2;
lamdate = 5/180*pi;
rangete = 50/180*pi;
 
obs_temp = obs;
lane_temp = deresolution(lane,gap*0.5);
size(lane_temp)/size(lane)

chk0 = zeros(size(lane_temp,1),1)~=0;
chk0_model = zeros(m,1);
chk1 = 0;chk2 = 0;chk3 = 0;chk4 = 0;

figure_on = 0;

%%
tic
for kk = 1 : 3
    kk
    rx = (xmax-xmin)*rand(n,1)+xmin;
    ry = (ymax-ymin)*rand(n,1)+ymin;
    rt = 2*rangete*rand(n,1)-rangete;
    
    iter = 0;
    
    
    weight_best(kk) = 0;
    bestpos(kk,:) = [sum(rx)/size(rx,1)  sum(ry)/size(ry,1) 0];
    
    for k = 1:50
        if(figure_on)
            figure(kk)
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
            
            %%% detect obstacles inside the parking line
            if(~isempty(obs_temp))
                chk1 = (anglebwlines(obs_temp,model_sub(model_idx(1),:),model_sub(model_idx(2),:))>0);
                chk2 = (anglebwlines(obs_temp,model_sub(model_idx(2),:),model_sub(model_idx(3),:))>0);
                chk3 = (anglebwlines(obs_temp,model_sub(model_idx(3),:),model_sub(model_idx(4),:))>0);
                chk4 = (anglebwlines(obs_temp,model_sub(model_idx(4),:),model_sub(model_idx(5),:))>0);
            end
            
            if(sum(chk1&chk2&chk3&chk4)>0)
                weight_sub = 0.0;
            else
                %%% calculate the particle's weight along the parking line
                
                weight_sub = 1.0;
                chk_model = chk0_model;
                
                for j = 1:m
                    weight_temp = (-1/gap).*sqrt(min((model_sub(j,1)-lane_temp(:,1)).^2+(model_sub(j,2)-lane_temp(:,2)).^2))+1;
                    if(weight_temp<0)
                        weight_temp = 1.0;
                    else
                        weight_temp = weight_temp + 1.0;
                        chk_model(j) = 1;
                    end
                    weight_sub = weight_sub * weight_temp;
                end
                weight_temp = ((sum(chk_model(1:5))>0)+(sum(chk_model(6:10))>0)+(sum(chk_model(17:21))>0)+(sum(chk_model(22:26))>0)+(sum(chk_model(11:16))>0)+(sum(chk_model(27:32))>0))+1;
                weight_sub = weight_sub * weight_temp;
                
            end
            
            weight(i,1) = weight_sub;
            
            if(figure_on)
                if(weight_sub>1.0)
                    plot(model_sub(:,1), model_sub(:,2),'c')
                else
                    plot(model_sub(:,1), model_sub(:,2),'y')
                end
            end
            
        end
        
        
        [weight_best_new,W_idx] = max(weight);
        if(weight_best_new>weight_best(kk))
            iter = 0;
            weight_best(kk) = weight_best_new;
            rotM = [cos(rt(W_idx)) -sin(rt(W_idx)); sin(rt(W_idx)) cos(rt(W_idx))];
            model_sub = (rotM*model')';
            model_sub(:,1) = model_sub(:,1) + rx(W_idx);
            model_sub(:,2) = model_sub(:,2) + ry(W_idx);
            bestpos(kk,:) = [rx(W_idx) ry(W_idx) rt(W_idx)];
            
        else
            iter = iter + 1;
            rotM = [cos(bestpos(kk,3)) -sin(bestpos(kk,3)); sin(bestpos(kk,3)) cos(bestpos(kk,3))];
            model_sub = (rotM*model')';
            model_sub(:,1) = model_sub(:,1) + bestpos(kk,1);
            model_sub(:,2) = model_sub(:,2) + bestpos(kk,2);
        end
        
        weight = weight./norm(weight);
        intweight = int32(n*weight);
        intweight = int32(n^2/sum(intweight)*weight);
        
        if(figure_on)
            plot(model_sub(:,1), model_sub(:,2),'k')
            plot(lane_temp(:,1),lane_temp(:,2),'r.')
            if(~isempty(obs))
                plot(obs_temp(:,1),obs_temp(:,2),'m.')
            end
            axis equal
            [k sum(intweight) max(intweight) iter]
            %         pause
        end
        
        if( max_iter <= iter)
            break
        end
        
        rx_new = [];
        ry_new = [];
        rt_new = [];
        
        for i = 1 : size(rx,1)
            rx_new = [rx_new ; rx(i) + (rand(intweight(i,1),1).*2-1).*lamdaxy];
            ry_new = [ry_new ; ry(i) + (rand(intweight(i,1),1).*2-1).*lamdaxy];
            rt_new = [rt_new ; rt(i) + (rand(intweight(i,1),1).*2-1).*lamdate];
        end
        if(np>0)
            rx_new = [rx_new ; bestpos(kk,1) + (rand(np,1).*2-1).*lamdaxy];
            ry_new = [ry_new ; bestpos(kk,2) + (rand(np,1).*2-1).*lamdaxy];
            rt_new = [rt_new ; bestpos(kk,3) + (rand(np,1).*2-1).*lamdate];
        end
        
        rx = rx_new;
        ry = ry_new;
        rt = rt_new;
    end
    obs_temp = [obs_temp; rx ry];
end
toc


figure()
if(~isempty(obs))
    plot(obs(:,1),obs(:,2),'m.')
end
hold on
plot(lane_temp(:,1),lane_temp(:,2),'r.')
for kk = 1:size(bestpos,1)
    rotM = [cos(bestpos(kk,3)) -sin(bestpos(kk,3)); sin(bestpos(kk,3)) cos(bestpos(kk,3))];
    model_sub = (rotM*model')';
    model_sub(:,1) = model_sub(:,1) + bestpos(kk,1);
    model_sub(:,2) = model_sub(:,2) + bestpos(kk,2);
    
    plot(model_sub(:,1), model_sub(:,2),'k')
end


axis equal
[~,kk] = max(weight_best);
rotM = [cos(bestpos(kk,3)) -sin(bestpos(kk,3)); sin(bestpos(kk,3)) cos(bestpos(kk,3))];
model_sub = (rotM*model')';
model_sub(:,1) = model_sub(:,1) + bestpos(kk,1);
model_sub(:,2) = model_sub(:,2) + bestpos(kk,2);

plot(model_sub(:,1), model_sub(:,2),'b')







