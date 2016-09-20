function out = inrange(data,line,drange)

ab = line(2,:)-line(1,:);
ca = [data(:,1)-line(2,1) data(:,2)-line(2,2)];
cb = [data(:,1)-line(1,1) data(:,2)-line(1,2)];

d = abs(ab(2)*data(:,1)-ab(1)*data(:,2)+line(2,1)*line(1,2)-line(2,2)*line(1,1))/sqrt(sum(ab.^2));

acb = [sum((ca.^2)')' sum((cb.^2)')'];
maxacb = max(acb')';
minacb = min(acb')';

out = d<=drange&minacb + sum(ab.^2) >= maxacb;

end
