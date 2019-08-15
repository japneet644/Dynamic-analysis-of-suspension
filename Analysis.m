function y = pattern(x)
l1c = [x(1),x(2),x(3)];l2c = [x(4),x(5),x(6)];

l3c = [x(7),x(8),x(9)];
l4c = [x(10),x(11),x(12)];
l5c = [x(13),x(14),x(15)];
l5u = [x(16),x(17),x(18)];
ubj = [x(19),x(20),x(21)];
lbj = [x(22),x(23),x(24)];
[SAI,castor,scrubr,trail,antid,fvich,svich,damperc,camber] = finallydone(l1c,l2c,l3c,l4c,l5c,l5u,ubj,lbj);
y = 4*(scrubr - 4.5)^2 + 2*(castor - 18)^2 +8*(SAI - 4)^2 + 2*(fvich - 60)^2 + (trail - 5)^2 + (antid - 6)^2 +6*(svich-14.5)^2+5*(damperc-5)+20*(camber+2);
end
function  [SAI,castor,scrubr,trail,antid,fvich,svich,damperc,camber] = finallydone(l1c,l2c,l3c,l4c,l5c,l5u,ubj,lbj)
% % x is along rearwards
% % l1c = [-14.978,24.752,25.996];
% % l2c = [14.669,24.456,27.610];
% % l4c = [-18.927,23.0,13.97];
% % l3c = [14.669,23.0,13.97];
% % l5c = [0,23.0,40.0];
% % %l6c = [10,20,30];
% % tc =  [0,62.5,29.21];
% % l5u = [1.4167,47.0153,13.97];
% %l6u = [10,20,30];
% a =100;
% %ubj = [0.373,57.602,29.2];
% %lbj = [2.125,59.023,13.97];
wheeltravel = zeros(1,'double');
% camber = zeros(1,a+1,'double');
i=-1;
a = ubj(2);
b = lbj(2);
lca =  planeintersection(l3c,l4c,[tc(1),tc(2),0],[1,0,0]);
c =  [lbj(1),lca(1);lbj(2),lca(2)]\[p5(1);p5(2)];
%p = c(1); q=c(2);
while wheeltravel < 10
% find ubj lbj
t =0.01;
i=i+1;
point3 = planeintersection(ubj,l1c,tc,[0,1,0]);
point4 = planeintersection(ubj,l2c,tc,[0,1,0]);
point4(2) = tc(2);
point3(2) = tc(2);
point14 = planeintersection(lbj,l3c,tc,[0,1,0]);
point13 = planeintersection(lbj,l4c,tc,[0,1,0]);
point14(2) = tc(2);
point13(2) = tc(2);

prubj = [tc(1) ,ubj(2), ubj(3)];
prlbj = [tc(1) ,lbj(2), lbj(3)];
uca =  planeintersection(l1c,l2c,[tc(1),tc(2),0],[1,0,0]);
lca =  planeintersection(l3c,l4c,[tc(1),tc(2),0],[1,0,0]);
uca(1) = tc(1);
lca(1) = tc(1);
svic  = intersection(point3, point4, point13, point14);
fvic  = intersection(prubj,uca,prlbj,lca);
axis = (-fvic + svic).*(1/sqrt(sum((fvic -svic).*(fvic-svic))));
svich = svic(3);
fvich = fvic(3);
%steeraxis = ubj - lbj;
mat =   [lbj;ubj;tc];
omega = [axis;axis;axis;];
mat2  = [svic;svic;svic;];
newc  = mat + cross(mat-mat2,omega,2).*t;
%newdamper = newc(1,:);
ubj = newc(2,:);
lbj = newc(1,:);
tc =  newc(3,:);
SAI = SAIF(ubj,lbj);
castor = castorf(ubj,lbj);
scrubr = scrubrf(ubj,lbj,tc(3)-29.21,tc(2));
trail =  trailf(ubj,lbj,tc(3)-29.21,tc(1));
antid = antidf(0.6,svic);
wheeltravel(i+1) = tc(3) - 29.21; 
%incomplete 
newdamper = newf(lbj,lca,c(1),c(2));
rcheight = rcheightf(tc,fvi) ;%cg height pending to be included

camber = atan((-lbj(2) + ubj(2)-(a-b))/(ubj(3) - lbj(3)))*180/pi;
damperc = sqrt(sum((newdamper-l5c).*(newdamper-l5c)))-sqrt(sum((l5u-l5c).*(l5u-l5c)));

end

end

function vec =intersection(a,b,c,d)
if(dot(a-b,[0,0,1])==0)
    y = [a(1)-c(1);a(2)-c(2)];
    A = [d(1)-c(1),a(1)-b(1);d(2)-c(2),a(2)- b(2)];
    s = pinv(A)*y;
    vec = c + vpa(s(2),3).*(d-c);
elseif(dot(a-b,[0,1,0])==0)
    y = [a(1)-c(1);a(3)-c(3)];
    A = [d(1)-c(1),a(1)-b(1);d(3)-c(3),a(3)- b(3)];
    s = pinv(A)*y;
    vec = c + vpa(s(2),3).*(d-c);
else
    y = [a(2)-c(2);a(3)-c(3)];
    A = [d(2)-c(2),a(2)-b(2);d(3)-c(3),a(3)- b(3)];
    s = pinv(A)*y;
    vec = c + vpa(s(2),3).*(d-c);
end

end


function A = planeintersection(a,b,c,n)
syms x
u = vpa(solve(sum(( a+x*(a-b) - c).*n )==0,x),3);
A = a + u*(a-b);
end
function A = newf(lbj,lca,b,a)
A = (b*lbj + a*lca )/(a+b);
end
function castor = castorf(ubj,lbj)
castor = atan( (-ubj(1)+ lbj(1))/(ubj(3)- lbj(3)) )*180/pi;
end

function SAI = SAIF(ubj,lbj)
SAI = atan( ( -ubj(2)+ lbj(2) )/( ubj(3)- lbj(3) ) )*180/pi;
end

function scrubr =scrubrf(ubj,lbj,z,tc)
t = vpa((z - lbj(3))/(lbj(3)- ubj(3)),3);
y = lbj(2)+t*(lbj(2)-ubj(2));
scrubr = tc - y;
end

function trail = trailf(ubj,lbj,z,tc)
t = (z - lbj(3))/(lbj(3)- ubj(3));
x = lbj(1) + t*(lbj(1)-ubj(1));
trail = x - tc;
end

function anti = antidf(bb,svic)
anti = bb*svic(3)/svic(1);
end

function h = rcheightf(tc,fvic)
t = -fvic(2)/(fvic(2) - tc(2));
h = fvic(3) + t*(fvic(3) - tc(3));
end


% function a =length(a,b)
% a = sqrt(sum((a-b).*(a-b)));
% end