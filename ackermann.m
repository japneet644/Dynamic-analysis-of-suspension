w = 1.19;
%d = 0.24;
k = 0.07684;
b = linspace(0,45,300)*pi/180;
%z = zeros(1,100);
x = linspace(0,pi/4,100);
error = zeros(1,300);
do = dack(w,1.52,x);
% for i = 1:300
%     z = relation(b(i));
%     error(i) = sqrt(sum((z-do).*(z-do)));
% end
% figure,plot(b*180/pi,error,'.');
% title('error'); 
figure,plot(x*180/pi,do*180/pi,'.');
hold on
z = relation(10*pi/180);
plot(x*180/pi,z*180/pi,'.');
% % % % % % % % z = relation(14*pi/180,k);
% % % % % % % % turiningr = sqrt((0.6517)^2 + 1.52*1.52*((cot(x) + cot(z)).*(cot(x) + cot(z)))/4 );
% % % % % % % % figure,plot(x(10:100)*(180)/pi,turiningr(10:100))
%z = @(x,y) (w - d*sin(x+b) - d * sin(b - y)).^2 + d*d*(cos(x+b) + cos(b-y) ).^2 - (w - 2*d*sin(b))



function do = dack(w,l,di)
do = acot(w/l + cot(di)); 
end

% function z = relation(b,k)
% z = zeros(1,100);
% x = linspace(0,pi/3,100);
% for i=1:100
      %sqrt((1+k*k - 2*k*sin(b+x(i))))
%     z(i) = b +acos( (2*sin(b)+k*cos(2*b)-sin(b+x(i)))/sqrt((1+k*k - 2*k*sin(b+x(i)))) ) - atan( (1-k*sin(b+x(i)))/(k*cos(b+x(i))) ); 
% end
function z = relation(b)
z = zeros(1,100);
t=100;
i = linspace(0,pi/4,t);
l =36;
w = 119;
a =(w-l)/2;
x =5;
d =36;
for u = 1:100
    q = (x*sin(i(u)+b) - a) + sqrt( (a - x*sin(b+i(u)))^2 + 2*x*( d*(-cos(b) + cos(b+i(u))) -  a*(-sin(b) + sin(b+i(u))) ) );
    (2*a*q/x -(a+q)*sin(b+i(u)) - d*cos(b+i(u)))/sqrt( (a-q)^2 + d^2 )
    z(u) = b + atan(d/(a-q)) + asin( (2*a*q/x -(a+q)*sin(b+i(u)) - d*cos(b+i(u)))/sqrt( (a-q)^2 + d^2 ) );
end

end
%z = @(x,y) sin(b+x)+sin(b-y)-2*sin(b)+(w/d)*(cos(b+x).*cos(b-y)-2*cos(2*b)); 
% for i=1:100
%     %sqrt((1+k*k - 2*k*sin(b+x(i))))
%     z(i) = b+acos( (2*sin(b)+k*cos(2*b)-sin(b+x(i)))/sqrt((1+k*k - 2*k*sin(b+x(i)))) ) - atan( (1-k*sin(b+x(i)))/(k*cos(b+x(i))) ); 
% end
% % %figure,fimplicit(z,[0,pi/3])
%  figure,plot(x,z,'r');
%  title('real case');
%  hold on
% b = 5*pi/180;
% 
% plot(x,z,'g');
% hold on
% b = 15*pi/180;
% for i=1:100
%     %sqrt((1+k*k - 2*k*sin(b+x(i))))
%     z(i) = b+ acos( (2*sin(b)+k*cos(2*b)-sin(b+x(i)))/sqrt((1+k*k - 2*k*sin(b+x(i)))) ) - atan( (1-k*sin(b+x(i)))/(k*cos(b+x(i))) ); 
% end
% plot(x,z,'b');
% hold on
% plot(x,do,'.');