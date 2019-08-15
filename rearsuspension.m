function X = rear()
%forces as compression are positive
%mote vale arm is arm 4
%%% look as points in diagram
% 6 points on upright as input as a 1*3 matrix [p1x p1y p1z] 
pc1 = [2 5.5 13].*2.54; %input('cordinates of point 1 on upright ');
pc2 = [2 5.5 18].*2.54; %input('cordinates of point 2 on upright '); 
pc3 = [-2 5.5 13].*2.54; %input('cordinates of point 3 on upright ');
pc4 = [-23 12.5 13].*2.54;  %input('cordinates of point 4 on upright ');
pc5 = [-23 12.5 17].*2.54;  %input('cordinates of point 5 on upright ');
pc6 = [51.52-73,14.71,26.59].*2.54;  %input('cordinates of point 6 on upright ');
%pc6 = [51.52-73,14.71,26.59].*2.54;
% 6 points on chasis as input as a 1*3 matrix 
p1 = [2 ,21, 8.5].*2.54;   %input('cordinates of point 1 on chasis');
p2 = [2 ,21, 13.5].*2.54;   %input('cordinates of point 2 on chasis'); 
p3 = [-2 ,21, 8.5].*2.54;   %input('cordinates of point 3 on chasis');
p4 = [-2.5 ,21, 10.5].*2.54;   %input('cordinates of point 4 on chasis');
p5 = [-2.5 ,21, 14.5].*2.54;   %input('cordinates of point 5 on chasis');
p6 = [61.96-73, 17.80, 11 ].*2.54;   %input('cordinates of point 6 on chasis');

g1 = [72.67-73,17.72,10.98].*2.54;
g2 = [70.33-73,18.93,11.02].*2.54;
u = (g2-g1)/sqrt(sum((g2 - g1).*(g2- g1)));
% cordinates of bottom of tire centre [x y 0]
t =[0 ,21 , 11.5].*2.54; %input('cordinates of points of tire ');
r  = 11.5*2.54;
%unit vectors representing links 1 2 3 4 5 6 as 1*3 matrix
vec1 = -(pc1 - p1)/sqrt(sum((pc1 - p1).*(pc1- p1)));
vec2 = -(pc2 - p2)/sqrt(sum((pc2 - p2).*(pc2- p2)));
vec3 = -(pc3 - p3)/sqrt(sum((pc3 - p3).*(pc3- p3)));
vec4 = -(pc4 - p4)/sqrt(sum((pc4 - p4).*(pc4- p4)));
vec5 = -(pc5 - p5)/sqrt(sum((pc5 - p5).*(pc5- p5)));
vec6 = cross(vec4,u)/sqrt(sum(cross(vec4,u)));

dampervec = (p6 - pc6)/sqrt(sum((pc6 - p6).*(pc6- p6)));
inc = dot(dampervec,[0,0,-1]);
ratio = (sqrt(sum((p4-pc4).*(p4-pc4))))/sqrt(sum((p6-pc4).*(p6-pc4)));

% forces as input fx is force in X direction
fx = [-1088*0.8, 0,    4420, 0,     1000];           %input('force in X direction ');
fy = [0,         0,    0,    -1200, 0];   %input('force in y direction ');
fz = [1088,      4000, 5190, 2050,  0];        %input('force in z direction ');

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

a1 = vec1(1)*(t(2) - p1(2)) - vec1(2)*(t(1) - p1(1));
a2 = vec2(1)*(t(2) - p2(2)) - vec2(2)*(t(1) - p2(1));
a3 = vec3(1)*(t(2) - p3(2)) - vec3(2)*(t(1) - p3(1));
a4 = vec4(1)*(t(2) - p4(2)) - vec4(2)*(t(1) - p4(1));
a5 = vec5(1)*(t(2) - p5(2)) - vec5(2)*(t(1) - p5(1));
a6 = vec6(1)*(t(2) - p6(2)) - vec6(2)*(t(1) - p6(1));
%a7 = pervec6(1)*(t(2) - p6(2)) - pervec6(2)*(t(1) - p6(1));

b1 = -vec1(1)*(t(3) - p1(3)) + vec1(3)*(t(1) - p1(1) );
b2 = -vec2(1)*(t(3) - p2(3)) + vec2(3)*(t(1) - p2(1) );
b3 = -vec3(1)*(t(3) - p3(3)) + vec3(3)*(t(1) - p3(1) );
b4 = -vec4(1)*(t(3) - p4(3)) + vec4(3)*(t(1) - p4(1) );
b5 = -vec5(1)*(t(3) - p5(3)) + vec5(3)*(t(1) - p5(1) );
b6 = -vec6(1)*(t(3) - p6(3)) + vec6(3)*(t(1) - p6(1) );
%b7 = -pervec6(1)*(t(3) - p6(3) ) + pervec6(3)*( t(1) - p6(1) );

c1 = vec1(3)*(p1(2) - t(2)) + vec1(2)*(t(3) - p1(3));
c2 = vec2(3)*(p2(2) - t(2)) + vec2(2)*(t(3) - p2(3));
c3 = vec3(3)*(p3(2) - t(2)) + vec3(2)*(t(3) - p3(3));
c4 = vec4(3)*(p4(2) - t(2)) + vec4(2)*(t(3) - p4(3));
c5 = vec5(3)*(p5(2) - t(2)) + vec5(2)*(t(3) - p5(3));
c6 = vec6(3)*(p6(2) - t(2)) + vec6(2)*(t(3) - p6(3));
%c7 = pervec6(1)*(p6(2) - t(2)) + pervec6(2)*(t(3) - p6(3));
%;
%coeficents of f1 f2 f3 ... f6 in equation 1...6
X = zeros(4,8);
 mat = [   vec1(1) vec2(1) vec3(1) vec4(1) vec5(1) vec6(1);
           vec1(2) vec2(2) vec3(2) vec4(2) vec5(2) vec6(2);
           vec1(3) vec2(3) vec3(3) vec4(3) vec5(3) vec6(3);
           a1      a2      a3      a4      a5      a6; 
           b1      b2      b3      b4      b5      b6;
           c1      c2      c3      c4      c5      c6;
       ];
for i  = 1:4
    %F = [-fx(i); -fy(i); -fz(i) ; 0; -fx(i)*r; 0];
    F = [-fx(i); -fy(i); -fz(i) ; -fz(i)*5; 0; fx(i)*5];
    %X is answer = [f1 f2 f3 f4 f5 f6]
    X(i,1:6) = pinv(mat)*F;
    X(i,7) = -ratio*X(i,6)/inc;
    X(i,8) = X(i,4) - X(i,7)*dot(vec4,dampervec);
    %along = X(4) + X(6)*dot(vec4,pvec6);
    %perpendicular = X(6)*(sqrt(1 - dot(vec4,pvec6))); 
end
%dot(vec4,dampervec)
   U = zeros(4,24);
   
   for i =1 :4
       j=1;
       U(i,j:j+2) = X(i,1)*vec1;
       j = j+3;
       U(i,j:j+2) = X(i,2)*vec2;
       j = j+3;
       U(i,j:j+2) = X(i,3)*vec3;
       j = j+3;
       U(i,j:j+2) = X(i,4)*vec4;
       j = j+3;
       U(i,j:j+2) = X(i,5)*vec5;
       j = j+3;
       U(i,j:j+2) = X(i,6)*vec6;
       j = j+3;
       U(i,j:j+2) = X(i,7)*dampervec;
       j = j+3;
       U(i,j:j+2) = X(i,7)*vec4;
       j = j+3;
   end
   %xlswrite(filename,X(2),1,'F50:F58');
   %xlswrite(filename,X(3),1,'G50:G58');
   %xlswrite(filename,X(4),1,'H50:H58');
   filename ='A.xlsx';
   xlswrite(filename,U,1,'A100:Z104');
   fclose('all');
end