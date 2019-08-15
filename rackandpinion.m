w = 1.11652;
l = 0.452;%%0.44958;
A = (w-l)/2;
x = 0.10;
iter = 300;
b = linspace(0,40,300)*pi/180;
%z = zeros(1,iter);
di = linspace(0,60*pi/180,iter);
error = zeros(1,300);
do = dack(w,1.47,di);
d = 0.0775;
figure,plot(di*180/pi,do*180/pi,'.');
for i = 1:iter
    y  = sqrt( (A - x*sin(b(i)))^2 + (d - x*cos(b(i)))^2 );
    z = relation(b(i),iter,A,x,di,y,d);
    error(i) = sqrt(sum((z-do).*(z-do)));
end

% b = 10*pi/180;
% y  = sqrt( (A - x*sin(b))^2 + (d - x*cos(b))^2 );
% z = relation(b,iter,A,x,di,y,d);
figure,plot(b*180/pi,error,'.');
title('error'); 
figure,plot(di*180/pi,do*180/pi);
hold on
b =25*pi/180;
y  = sqrt( (A - x*sin(b))^2 + (d - x*cos(b))^2 );
z = relation(b,iter,A,x,di,y,d);
plot(di*180/pi,z*180/pi,'.');

function do = dack(w,l,di)
do = acot(w/l + cot(di)); 
end

% function z = relation(b,iter,A,x,di,y,d)
% z = zeros(1,iter);
% q = zeros(1,iter);
% for u = 1:iter
%     q(u) = x*sin(di(u)+b) - A + sqrt( y^2 - ( d-x*cos(di(u)+b) )^2 );
%     c = ( y^2 - d^2 - x^2 - (A - q(u))^2 )/(2*x);
%     z(u) = b + asin( c*(A-q(u)) + sqrt( (c*(A-q(u)))^2 - ( (A-q(u))^2 + d^2 )*(c^2 - d^2) ) );
% end
% 
% end
function z = relation(b,iter,A,x,di,y,d)
z = zeros(1,iter);
q = zeros(1,iter);
for u = 1:iter
    q(u) = x*sin(di(u)+b) - A + sqrt( y^2 - ( d-x*cos(di(u)+b) )^2 );
    c = ( y^2 - d^2 - x^2 - (A - q(u))^2 )/(2*x*sqrt((A-q(u))^2 + d^2));
    z(u) = atan(d/(A-q(u)))+b+ asin(c);
end
end