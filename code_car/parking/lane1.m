% ccc
%
% load('partilane7.mat');
%%%%%
cd C:\autocarnav3\code_car\parking
Lsho = 2.1;%*3+3.3;
dLsho = 0.1;%*4;
Llon = 5.3;
dLlon = 0.1;
drange = 0.15;

L = (sqrt((Lsho-dLsho)^2+(Llon-dLlon)^2)+sqrt((Lsho+dLsho)^2+(Llon+dLlon)^2))*0.5;
dL = abs(sqrt((Lsho-dLsho)^2+(Llon-dLlon)^2)-sqrt((Lsho+dLsho)^2+(Llon+dLlon)^2))*0.5;
h = ((Lsho-dLsho)*(Llon-dLlon)/(L+dL)+(Lsho+dLsho)*(Llon+dLlon)/(L-dL))*0.5;1.95;
dh = abs((Lsho-dLsho)*(Llon-dLlon)/(L+dL)-(Lsho+dLsho)*(Llon+dLlon)/(L-dL))*0.5;


[lane_temp, ~] = deresolution(lane,drange,0,0);

plot(lane(:,1),lane(:,2),'m.')
hold on
plot(lane_temp(:,1),lane_temp(:,2),'.')
size(lane_temp,1)
axis equal
hold on
if(~isempty(obs))
    plot(obs(:,1),obs(:,2),'r.')
    [obs, ~] = deresolution(obs,drange,0,0);
    
end
A = lane_temp(1,:);
lane_temp = lane_temp(2:end,:);
lane_chkd = [];
lane_seed = [0 0 0 0 0 0 0 0 0];
count = 0;

tic
while(size(lane_temp,1)>0&&toc<5)
    
    b = sqrt((lane_temp(:,1) - A(1)).^2+(lane_temp(:,2) - A(2)).^2);
    B = sortrows([b(:,1)>=(L-dL)&b(:,1)<=(L+dL) lane_temp]);
    if(sum(B(:,1))>0)
        count = count + 1;
        lane_chkd(count,:) =  lane_seed;
        for j = size(B,1): -1 : size(B,1)-sum(B(:,1))+1
            ab = B(j,2:3)-A;
            d = abs(ab(2)*B(:,2)-ab(1)*B(:,3)+B(j,2)*A(2)-B(j,3)*A(1))/sqrt(sum(ab.^2));
            e = abs((B(:,2)-A(1)).^2+(B(:,3)-A(2)).^2+(B(:,2)-B(j,2)).^2+(B(:,3)-B(j,3)).^2-ab(1).^2-ab(2).^2);
            C = sortrows([d>=(h-dh)&d<=(h+dh)&e<0.5 B(:,2:3)]);
            if(sum(C(:,1))>0)
                for i = size(C,1): -1 :size(C,1)-sum(C(:,1))+1
                    if(anglebwlines(B(j,2:3),A,C(i,2:3))>0)
                        lane_sub = [0 A C(i,2:3) B(j,2:3) B(j,2:3)-C(i,2:3)+A A];
                    else
                        lane_sub = [0 A B(j,2:3)-C(i,2:3)+A B(j,2:3) C(i,2:3) A];
                    end
                    chkA = zeros(size(B,1),1)==1;
                    chkB = zeros(size(B,1),1)==0;
                    chkobs = zeros(size(obs,1),1)==0;
                    for k = 1:4
                        chkA = chkA | inrange(B(:,2:3),[lane_sub(k*2:k*2+1);lane_sub((k+1)*2:(k+1)*2+1)],drange);
                        chkA = chkA | (sum([B(:,2)-lane_sub(k*2) B(:,3)-lane_sub(k*2+1)]'.^2))'<=drange^2;
                        chkB = chkB&(anglebwlines(B(:,2:3),lane_sub(k*2:k*2+1),lane_sub((k+1)*2:(k+1)*2+1))>0);
                        if(~isempty(obs))
                            chkobs = chkobs&(anglebwlines(obs,lane_sub(k*2:k*2+1),lane_sub((k+1)*2:(k+1)*2+1))>0);
                        end
                    end
                    
                    lane_sub(1) = sum(chkA)-sum(chkB)+sum(chkA&chkB);
                    if((lane_sub(1)>=lane_chkd(count,1))&&(sum(chkobs)==0))
                        lane_chkd(count,:) = lane_sub(1:9);
                    end
                    
                end
            end
        end
    end
    
    A = lane_temp(1,:);
    lane_temp = lane_temp(2:end,:);
end
lane_out = [];
if(~isempty(lane_chkd))
    lane_chkd = sortrows(lane_chkd);
    [~,lane_idx] = max(lane_chkd(:,1)>=(sum(lane_chkd(:,1))/(size(lane_chkd,1)-sum(lane_chkd(:,1)==0))*0.5+lane_chkd(end,1)*0.5)&lane_chkd(:,1)>50);
    lane_chkd = lane_chkd(count : -1 : lane_idx,:);
    nsum = sum(lane_chkd(:,1));
    weight_max = lane_chkd(1,1);
    if(nsum>100&&weight_max>=50)
        for i = 1:size(lane_chkd,1)
            plot(lane_chkd(i,[2,4,6,8,2]),lane_chkd(i,[3,5,7,9,3]),'k')
            lane_chkd(i,2:end) = lane_chkd(i,2:end).* lane_chkd(i,1);
        end
        if(size(lane_chkd,1)>1)
            lane_out = sum(lane_chkd(:,2:end))/nsum;
        else
            lane_out = lane_chkd(:,2:end)/nsum;
        end
        lane_out = [weight_max lane_out];
        plot(lane_out([2,4,6,8,2]),lane_out([3,5,7,9,3]),'r')
    end
end
toc