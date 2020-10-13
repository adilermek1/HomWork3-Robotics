function q = IK_hw3(T,d1,a2,d3)

Pos=T(1:3,4);

px = Pos(1);
py = Pos(2);
pz = Pos(3);

if px == 0 && py ==0 
    fprintf('there are many solutions for theta1')
else 
 
end
fprintf('Solution for theta1')
    theta11 = atan(py/px)+(pi/2)
    theta12 = atan(py/px)+(pi/2)+pi



r = sqrt(px^2+py^2);
s = pz - d1;
fprintf('solution for theata2')
theta21 = atan(s/r)
theta22 = pi - atan(s/r)
fprintf('solutions for d3')
d33 = (sqrt(r^2+s^2))-a2









