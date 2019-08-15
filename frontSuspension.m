function X = front()
%forces as compression are positive
%%% look as points in diagram

pc1 = [38.528,23.480,50.572];%upper aft
pc2 = [13.077,25.400,50.572];%upper fore
pc3 = [7.997,23.480,33.020];%lower fore
pc4 = [38.528,25.400,33.020];%lower aft
pc5 = [34.4,25.00,66.50];%damper chasis end
pc6 = [30.943,22.362,28.0];%TIE rod
r =29.21;
p1 = [27.156,56.847,37.986];
p2=p1;
p3 = [25,58.768,20.434];
p4=p3;
t =  [26.078,63.5,0];
 p5 = [31.2,42.0,30.2];
 p6 = [30.943,57.646,16.78];
 
a = (pc4 + pc3)*0.5;
ratio = (sqrt(sum((p4-a).*(p4-a))))/sqrt(sum((p5-a).*(p5-a)));

% pc5 = p4 + (pc5-p5);
% p5 = p4;

%equations by balancing forces in X Y Z direction
% -fx = f1x + f2x + f3x +f4x +f5x +f6x
% -fy = f1y + f2y + f3y +f4y +f5y +f6y
% -fz = f1z + f2z + f3z +f4z +f5z +f6z

%equations by balancing torque in Z direction;
% TZ  0 = Sum(fiX*(t(2) - pi(2)) - Sum( fiy*(t(1) - pi(1)) )
% TY  0 = Sum( fiX*pi(3) ) - Sum( fiZ*( t(1) - pi(1) ) )
% TX  0 = SUM( fiZ*( pi(2) - t(2) ) - Sum( fiY*pi(3) )
% mat

%variables as coeficients (ignore)
vec1 = (p1 - pc1)/sqrt(sum((pc1 - p1).*(pc1- p1)));
vec2 = (p2 - pc2)/sqrt(sum((pc2 - p2).*(pc2- p2)));
vec3 = [1,0,0]; linkvec3 = (p3 - pc3)/sqrt(sum((pc3 - p3).*(pc3- p3)));
vec4 = [0,1,0]; linkvec4 = (p4 - pc4)/sqrt(sum((pc4 - p4).*(pc4- p4)));
vec5 = [0,0,1]; linkvec5 = (p5 - pc5)/sqrt(sum((pc5 - p5).*(pc5- p5)));
vec6 = (p6 - pc6)/sqrt(sum((pc6 - p6).*(pc6- p6)));
inc = abs(dot(linkvec5,cross(linkvec3,linkvec4)/norm(cross(linkvec3,linkvec4))))
plane= dot(cross(linkvec3,linkvec4),linkvec5)/norm(cross(linkvec3,linkvec4));

%vec4 = vec4 + vec4.* dot(vec4,vec6);
%dot(vec4,pvec6)
%vec6 = vec4.* dot(vec4,pvec6);
%pervec6 = pvec6 - vec4.*dot(vec4,pvec6);
% forces as input fx is force in X direction
fx = [731.68,0,    4420, 0  ,1000];           %input('force in X direction ');
fy = [0,     0,    0,    800,0];            %input('force in y direction ');
fz = [914.58,7500, 5004, 1413,0]; %input('force in z direction ');

%equations by balancing forces in X Y Z direction
% -fx = f1x + f2x + f3x +f4x +f5x +f6x
% -fy = f1y + f2y + f3y +f4y +f5y +f6y
% -fz = f1z + f2z + f3z +f4z +f5z +f6z

%equations by balancing torque in Z direction;
% TZ  0 = Sum(fiX*(t(2) - pi(2)) - Sum( fiy*(t(1) - pi(1)) )
% TY  0 = Sum( fiX*pi(3) ) - Sum( fiZ*( t(1) - pi(1) ) )
% TX  0 = SUM( fiZ*( pi(2) - t(2) ) - Sum( fiY*pi(3) )
% mat 
%trail = trailf(p1,p3,0,t(1));
%scrub = scrubrf(p1,p3,0,t(2));
%variables as  coeficients (ig;nore)
a1 = vec1(1)*(t(2) - p1(2)) - vec1(2)*(t(1) - p1(1));
a2 = vec2(1)*(t(2) - p2(2)) - vec2(2)*(t(1) - p2(1));
a3 = vec3(1)*(t(2) - p3(2)) - vec3(2)*(t(1) - p3(1));
a4 = vec4(1)*(t(2) - p4(2)) - vec4(2)*(t(1) - p4(1));
a5 = vec5(1)*(t(2) - p5(2)) - vec5(2)*(t(1) - p5(1));
a6 = vec6(1)*(t(2) - p6(2)) - vec6(2)*(t(1) - p6(1));

b1 = -vec1(1)*(t(3) - p1(3)) + vec1(3)*(t(1) - p1(1) );
b2 = -vec2(1)*(t(3) - p2(3)) + vec2(3)*(t(1) - p2(1) );
b3 = -vec3(1)*(t(3) - p3(3)) + vec3(3)*(t(1) - p3(1) );
b4 = -vec4(1)*(t(3) - p4(3)) + vec4(3)*(t(1) - p4(1) );
b5 = -vec5(1)*(t(3) - p5(3)) + vec5(3)*(t(1) - p5(1) );
b6 = -vec6(1)*(t(3) - p6(3)) + vec6(3)*(t(1) - p6(1) );
%b7 = -pervec6(1)*(t(3) - p6(3) ) + pervec6(3)*( t(1) - p6(1) );

c1 = vec1(3)*(p1(2) - t(2))  + vec1(2)*(t(3) - p1(3));
c2 = vec2(3)*(p2(2) - t(2)) + vec2(2)*(t(3) - p2(3));
c3 = vec3(3)*(p3(2) - t(2)) + vec3(2)*(t(3) - p3(3));
c4 = vec4(3)*(p4(2) - t(2)) + vec4(2)*(t(3) - p4(3));
c5 = vec5(3)*(p5(2) - t(2)) + vec5(2)*(t(3) - p5(3));
c6 = vec6(3)*(p6(2) - t(2)) + vec6(2)*(t(3) - p6(3));
%c7 = -pervec6(1)*(p6(2) - t(2)) - pervec6(2)*(t(1) - p6(1));
gamma = atan((a(3) -p3(3))/(p3(2) - a(2)))
rot = [1,0,0;0,cos(gamma),-sin(gamma);0,sin(gamma),cos(gamma)]
%coeficents of f1 f2 f3 ... f6 in equation 1...6
mat = [vec1(1) vec2(1) vec3(1) vec4(1) vec5(1) vec6(1);
       vec1(2) vec2(2) vec3(2) vec4(2) vec5(2) vec6(2);
       vec1(3) vec2(3) vec3(3) vec4(3) vec5(3) vec6(3);
         a1      a2      a3      a4      a5      a6; 
         b1      b2      b3      b4      b5      b6;
         c1      c2      c3      c4      c5      c6;
       ];
   X = zeros(4,14);
   u = [linkvec3(1),linkvec4(1);linkvec3(2),linkvec4(2)]; 
   g = zeros(4,3);
for i = 1:4
   
   F = [-fx(i); -fy(i); -fz(i); 0; 0; 0];
%X is answer = [f1 f2 f3 f4 f5 f6]   
   X(i,1:6) = pinv(mat)*F;
   X(i,7:9) = (rot*X(i,3:5)')';
   X(i,10) = -ratio*X(i,9)/inc;
   g = u\[-X(i,3);-(X(i,4))]
   X(i,11) = g(1);
   X(i,12) = X(i,11) - X(i,10)*sqrt(1 - plane^2);
   X(i,13) = g(2);
   X(i,14) = X(i,13) - X(i,10)*sqrt(1 - plane^2);
   %tierod = (fx(i)*scrub + fy(i)*trail)/((p6(1) - t(1))*cos(atan((pc6(3) - p6(3))/(p6(2) - pc6(2)))))
end

   
   U = zeros(4,24);
   for i =1 :4
       j=1;
       U(i,j:j+2) = X(i,1)*vec1;
       j = j+3;
       U(i,j:j+2) = X(i,2)*vec2;
       j = j+3;
       U(i,j) = X(i,3)*vec3(1);
       j = j+1;
       U(i,j) = X(i,4)*vec4(2);
       j = j+1;
       U(i,j) = X(i,5)*vec5(3);
       j = j+1;
       U(i,j:j+2) = X(i,6)*vec6;
       j = j+3;
       U(i,j:j+2) = X(i,10)*linkvec5;
       j = j+3;
       U(i,j:j+2) = X(i,11)*linkvec3;
       j = j+3;
       U(i,j:j+2) = X(i,12)*linkvec3;
       j = j+3;
       U(i,j:j+2) = X(i,13)*linkvec4;
       j = j+3;
       U(i,j:j+2) = X(i,14)*linkvec4;
       %j = j+3;
   end
%    U
%    filename ='A.xlsx';
%    xlswrite(filename,X,1,'A70:Z75');
%    xlswrite(filename,U,1,'A80:AJ85');
% % %    %xlswrite(filename,X(2),1,'F50:F58');
% % %    %xlswrite(filename,X(3),1,'G50:G58');
% % %    %xlswrite(filename,X(4),1,'H50:H58');
%    fclose('all');
end

function trail = trailf(ubj,lbj,z,tc)
t = (z - lbj(3))/(lbj(3)- ubj(3));
x = lbj(1) + t*(lbj(1)-ubj(1));
trail = x - tc;
end

function scrubr =scrubrf(ubj,lbj,z,tc)
t = (z - lbj(3))/(lbj(3)- ubj(3));
y = lbj(2)+t*(lbj(2)-ubj(2));
scrubr = tc - y;
end
