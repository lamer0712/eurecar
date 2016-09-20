function [out, map] = deresolution(in,gaps,incre,decre)

[val,trn] = min(size(in));
% if(trn~=2&&val~=1)
%     in = in';
% end
if(val==2)
    minmax(1,:) = min(in);
    minmax(2,:) = max(in);
    
    map = zeros(ceil((minmax(2,:)-minmax(1,:))/gaps));
    if(isempty(map))
        map = 1;
    end
    for i = 1 : size(in,1)
        idx = int32((in(i,:)-minmax(1,:))/gaps);
        if(sum(idx==0)==0)
            map(idx(1),idx(2)) = 1;
            if(incre)
                if(idx(1)-1>0)
                    map(idx(1)-1,idx(2)) = 1;
                end
                if(idx(2)-1>0)
                    map(idx(1),idx(2)-1) = 1;
                end
                map(idx(1),idx(2)+1) = 1;
                map(idx(1)+1,idx(2)) = 1;
            end
        end
    end
    
    if(decre)
        map2 = zeros(size(map));
        for i = 2 : size(map,1)-1
            for j = 2 : size(map,2)-1
                if(map(i,j)==1)
                    if(map(i-1,j)==1&&map(i,j-1)==1&&map(i+1,j)==1&&map(i,j+1)==1)
                        map2(i,j) = 1;
                    end
                end
            end
        end
        map = map2;
    end
    
    out = zeros(sum(sum(map)),2);
    if(~isempty(out))
        count = 0;
        for i = 1 : size(map,1)
            for j = 1 : size(map,2)
                if(map(i,j)==1)
                    count = count + 1;
                    out(count,:) = [i j]*gaps+minmax(1,:);
                end
            end
        end
    else
        out = in;
    end
else
    out = in;
    map = 1;
end


% plot(in(:,1),in(:,2),'.')
% hold on
% plot(out(:,1),out(:,2),'r.')
% axis equal

