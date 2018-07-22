function  [SAI,castor,scrubr,trail,antid,damperc,fvich,svich,change,wheeltravel] = finallydone()
% x is along rearwards
l1c = [-14.978,24.752,25.996];
l2c = [14.669,24.456,27.610];
l4c = [-18.927,23.0,13.97];
l3c = [14.669,23.0,13.97];
l5c = [0,23.0,40.0];
%l6c = [10,20,30];
tc =  [0,62.5,29.21];
l5u = [1.4167,47.0153,13.97];
%l6u = [10,20,30];
a =30;
ubj = [0.373,57.602,29.2];
lbj = [2.125,59.023,13.97];
SAI = zeros(1,a+1,'double');
scrubr = zeros(1,a+1,'double');
castor = zeros(1,a+1,'double');
trail = zeros(1,a+1,'double');
damperc = zeros(1,a+1,'double');
antid = zeros(1,a+1,'double');
fvich = zeros(1,a+1,'double');
svich = zeros(1,a+1,'double');
change = zeros(4,a+1,'double');
wheeltravel = zeros(1,a+1,'double');
camber = zeros(1,a+1,'double');
i=-1;
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
%svic = intersection( [0,l1c(2),l1c(3)],[0,l2c(2),l2c(3)],[0,l3c(2),l3c(3)],[0,l4c(2),l4c(3)] );
%fvic = intersection( [l1c(1),0,l1c(3)],[l2c(1),0,l2c(3)],[l3c(1),0,l3c(3)],[l4c(1),0,l4c(3)] );
%axis = (fvic - svic)/sqrt((fvic-svic).*(fvic-svic));
svic  = intersection(point3, point4, point13, point14);
fvic  = intersection(prubj,uca,prlbj,lca);
if(fvic == [inf,inf,inf])
    axis = [0,-1,0]; 
elseif(svic ==[inf,inf,inf]) %#ok<*BDSCA>
    axis = [1,0,0];
else
    axis = (-fvic + svic).*(1/sqrt(sum((fvic -svic).*(fvic-svic))));
end

svich(i+1) = svic(3);
fvich(i+1) = fvic(3);
%steeraxis = ubj - lbj;
mat =   [lbj;ubj;tc];
omega = [axis;axis;axis;];
mat2  = [fvic;fvic;fvic;];
newc  = mat + cross(mat-mat2,omega,2).*t;
%newdamper = newc(1,:);
ubj = newc(2,:);
lbj = newc(1,:);
tc =  newc(3,:);
SAI(i+1) = SAIF(ubj,lbj);
castor(i+1) = castorf(ubj,lbj);
scrubr(i+1) = scrubrf(ubj,lbj,tc(3)-29.21,tc(2));
trail(i+1) =  trailf(ubj,lbj,tc(3)-29.21,tc(1));
antid(i+1) = antidf(0.6,svic);
wheeltravel(i+1) = tc(3) - 29.21; 
%incomplete 
newdamper = newf(lbj,lca);
%rcheight = rcheightf(tc,fvi;c) ;%cg height pending to be included

camber(i+1) = atan((lbj(2) - ubj(2)-1.421)/(ubj(3) - lbj(3)))*180/pi;
damperc(i+1) = dampercf(newdamper-l5c,l5u-l5c);
%
change(1,i+1) = length(ubj,l1c);
change(2,i+1) = length(ubj,l2c);
change(3,i+1) = length(lbj,l3c);
change(4,i+1) = length(lbj,l4c);
end
% figure ,plot(wheeltravel(1:i),change(1,1:i),'.')
% title('link 1 vs wheeltravel');
% figure ,plot(wheeltravel(1:i),change(2,1:i),'.')
% title('link 2 vs wheeltravel');
% figure ,plot(wheeltravel(1:i),change(3,1:i),'.')
% title('link 3'); 
% figure ,plot(wheeltravel(1:i),change(4,1:i),'.')
% title('link 4');
% figure ,plot(wheeltravel(1:i),damperc(1:i))
% title('dampercompression vs wheeltravel ');
% figure ,plot(wheeltravel(1:i),SAI(1:i))
% title('SAI vs wheeltravel');
% figure ,plot(wheeltravel(1:i),scrubr(1:i))
% title('Scrub Radius vs wheeltravel');
% figure ,plot(wheeltravel(1:i),svich(1:i))
% title('Sv IC height vs wheeltravel');
% figure ,plot(wheeltravel(1:i),fvich(1:i))
% title('fv IC height vs wheeltravel');
% figure, plot(wheeltravel(1:i),castor(1:i))
% title('castor vs wheeltravel');
% figure , plot(wheeltravel(1:i),camber(1:i))
% title('Camber vs Wheeltravel initial camber is 0');
end

function vec =intersection(a,b,c,d)
if (cros(a-b,c-d)==0)
    vec = [inf,inf,inf];

else
    if(dot(a-b,[0,0,1])==0)
        y = [a(1)-c(1);a(2)-c(2)];
        A = [d(1)-c(1),a(1)-b(1);d(2)-c(2),a(2)- b(2)];
        s = pinv(A)*y;
        vec = c + vpa(s(2),4).*(d-c);
    else
        if(dot(a-b,[0,1,0])==0)
            y = [a(1)-c(1);a(3)-c(3)];
            A = [d(1)-c(1),a(1)-b(1);d(3)-c(3),a(3)- b(3)];
            s = pinv(A)*y;
            vec = c + vpa(s(2),4).*(d-c);
        else
            y = [a(2)-c(2);a(3)-c(3)];
            A = [d(2)-c(2),a(2)-b(2);d(3)-c(3),a(3)- b(3)];
            s = pinv(A)*y;
            vec = c + vpa(s(2),4).*(d-c);
        end
    end
end

end


function A = planeintersection(a,b,c,n)
syms x
u = vpa(solve(sum(( a+x*(a-b) - c).*n )==0,x),3);
A = a + u*(a-b);
end

function A = newf(lbj,lca)
a=1;
b=2;
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

function anti = antidf(svic)
anti = 3*svic(3)/svic(1);
end

function c = dampercf(new,old)
c = sqrt(sum(old.*old)) - sqrt(sum(new.*new));
end

function a =length(a,b)
a = sqrt(sum((a-b).*(a-b)));
end