ccc

load('partilane1.mat');
load('model.mat');

n = 100;
m = size(model,1);
mu = 0;%int32(n*0.02);

xmin = min(lane(:,1));
xmax = max(lane(:,1));
ymin = max(0,min(lane(:,2)));
ymax = max(lane(:,2));

lamdaxy = 0.225/2;
lamdate = 2/180*pi;
rangete = 50/180*pi;

rx = [];%(xmax-xmin)*rand(n,1)+xmin;
ry = [];%(ymax-ymin)*rand(n,1)+ymin;

while(size(rx,1)<n)
    rx_sub = (xmax-xmin)*rand(1)+xmin;
    ry_sub = (ymax-ymin)*rand(1)+ymin;
    if(sqrt(min((lane(:,1)-rx_sub).^2+(lane(:,2)-ry_sub).^2))>gap*2)
        rx = [rx ; rx_sub];
        ry = [ry ; ry_sub];
    end
end

rt = 2*rangete*rand(n,1)-rangete;
nn = 0;
chk0 = zeros(size(lane,1),1)~=0;
chk0_model = zeros(m,1);
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
        
        
        %%% detect obstacles inside the parking line
        chk1 = (anglebwlines(obs,model_sub(model_idx(1),:),model_sub(model_idx(2),:))>0);
        chk2 = (anglebwlines(obs,model_sub(model_idx(2),:),model_sub(model_idx(3),:))>0);
        chk3 = (anglebwlines(obs,model_sub(model_idx(3),:),model_sub(model_idx(4),:))>0);
        chk4 = (anglebwlines(obs,model_sub(model_idx(4),:),model_sub(model_idx(5),:))>0);
        
        if(sum(chk1&chk2&chk3&chk4)>0)
            weight_sub = 0.0;
        else
            %%% calculate the particle's weight along the parking line
            if(sqrt(min((lane(:,1)-rx(i)).^2+(lane(:,2)-ry(i)).^2))>gap*2)
                weight_sub = 1.0;
                chk_model = chk0_model;
                for j = 1:m
                    weight_temp = (-1/gap).*sqrt(min((model_sub(j,1)-lane(:,1)).^2+(model_sub(j,2)-lane(:,2)).^2))+1;
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
            else
                weight_sub = 0.0;
            end            
        end
        
        weight(i,1) = weight_sub;
        
        if(figure_on)
            if(weight_sub>=1.0)
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
        %         figure(2)
        %         subplot(2,1,1)
        %         plot(weight)
        %         subplot(2,1,2)
        %         plot(intweight,'.')
        [k sum(intweight) nn]
        %         pause
    end
end
toc












