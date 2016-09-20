function out = get_radius(x, y)

kk = 1;
x_new = [];
y_new = [];

for k = 1 : size(x,2)-2
%     
    c = 2.0*(x(k)*(y(k+1) - y(k+2)) + x(k+1)*(y(k+2) - y(k)) + x(k+2)*(y(k) - y(k+1)));
    a = (x(k)*x(k) + y(k)*y(k))*(y(k+1) - y(k+2)) + (x(k+1)*x(k+1) + y(k+1)*y(k+1))*(y(k+2) - y(k)) + (x(k+2)*x(k+2) + y(k+2)*y(k+2))*(y(k) - y(k+1));
    b = (x(k)*x(k) + y(k)*y(k))*(x(k+2) - x(k+1)) + (x(k+1)*x(k+1) + y(k+1)*y(k+1))*(x(k) - x(k+2)) + (x(k+2)*x(k+2) + y(k+2)*y(k+2))*(x(k+1) - x(k));
    c = 1.0 / c;
    a = a * c;
    b = b * c;
    r(k) = sqrt((x(k+1) - a)*(x(k+1) - a) + (y(k+1) - b)*(y(k+1) - b));
%         r(k) = ((1+(3*ab(1)*x(k)^2 + 2*ab(2)*x(k))^2)^(1.5))/(6*ab(1)*x(k)+2*ab(2))
    
    if(abs(r(k)) < 3.8)
        x_new(kk) = x(k+1);
        y_new(kk) = y(k+1);
        kk = kk+1;
        if(kk>2);
            kk = 2;
        end
    end
end

out = [x_new ; y_new];