function theta = anglebwlines(o,m1,m2)
a = [ m2(1)-m1(1)  m2(2)-m1(2)];
b = [o(:,1)-m1(1) o(:,2)-m1(2)];
axb = a(1)*b(:,2)-a(2)*b(:,1);
theta = asin((axb)./(norm(a)*norm(b)));

