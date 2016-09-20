function circle(x,y,r)

theta = [0 : 1*pi/180 : 2*pi];

[x1, y1] = pol2cart(theta,r);

plot(x+x1,y+y1,'-y');