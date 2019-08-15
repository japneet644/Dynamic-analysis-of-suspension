function [X] = force_distriution_connected_links()
%forces as compression are positive
%%% look as points in diagram
% 6 points on upright as input as a 1*3 matrix [p1x p1y p1z]
pc1 = [190.5,  13.97, 33.02]; 
pc2 = [190.5,  13.97, 45.72]; 
pc3 = [180.34, 13.97, 33.02];
pc4 = [128.270, 33.02, 33.020];
pc5 = [128.270, 33.02, 40.640];
pc6 = [128.070, 41.550,67.460];
% 5 points on chasis as input as a 1*3 matrix 
p1 = [180.34 ,53.342, 21.59]; % lower suspension link 1
p2 = [180.34 ,53.342, 21.59]; % lower suspension link 2
p3 = [170.10 ,52.41, 24.906]; % pull rod
p4 = [166.355,52.41, 24.906]; % upper suspension link 1
p5 = [166.355,52.41, 24.906]; % upper suspension link 2 
p6 = [164.185, 48.406, 36.247]; % tie rod
% ratio = (sqrt(sum((p4-pc4).*(p4-pc4))))/sqrt(sum((p6-pc4).*(p6-pc4)));
% cordinates of bottom of tire centre [x y 0]
t =[170.18 ,58 ,29.21 ]; %input('cordinates of points of tire ');
r  = 11.5*2.54;
%unit vectors representing links 1 2 3 4 5 6 as 1*3 matrix
vec1 = (p1 - pc1)/sqrt(sum((pc1 - p1).*(pc1- p1)));
vec2 = (p2 - pc2)/sqrt(sum((pc2 - p2).*(pc2- p2)));
vec3 = (p3 - pc3)/sqrt(sum((pc3 - p3).*(pc3- p3)));
vec4 = (p4 - pc4)/sqrt(sum((pc4 - p4).*(pc4- p4)));
vec5 = (p5 - pc5)/sqrt(sum((pc5 - p5).*(pc5- p5)));
vec6 = (p6 - pc6)/sqrt(sum((pc6 - p6).*(pc6- p6)));
%vec4 = vec4 + vec4.* dot(vec4,vec6);
%dot(vec4,pvec6)
%vec6 = vec4.* dot(vec4,pvec6);
%pervec6 = pvec6 - vec4.*dot(vec4,pvec6);
Fx = [-1088*0.8, 0,    4420, 0,     0];
Fy = [0,         0,    0,    -1200, 0];
Fz = [1088,      4000, 4190, 2050,  500];
% Fx = f1x + f2x + f3x + f4x + f5x + f6x;
% Fy = f1y + f2y + f3y + f4y + f5y + f6y;
% Fz = f1z + f2z + f3z + f4z + f5z + f6z;

f1x = [1,0,0]; % 4,5 are upper links
f1y = [0,1,0];
f1z = [0,0,1];

f1a = (vec1 + vec2)/(norm(vec1 + vec2));
f1b = (vec1 - vec2)/(norm(vec1 - vec2));

f1t = vec6;


% a1 = sqrt(power((p1(2)+ pc1(2))/2 - t(2),2) + power((p1(3) + pc1(3))/2 - t(3),2));
% a2 = sqrt(power((p2(2)+ pc2(2))/2 - t(2),2) + power((p2(3) + pc2(3))/2 - t(3),2));
% a3 = sqrt(power((p3(2)+ pc3(2))/2 - t(2),2) + power((p3(3) + pc3(3))/2 - t(3),2));
% a4 = sqrt(power((p4(2)+ pc4(2))/2 - t(2),2) + power((p4(3) + pc4(3))/2 - t(3),2));
% a5 = sqrt(power((p5(2)+ pc5(2))/2 - t(2),2) + power((p5(3) + pc5(3))/2 - t(3),2));
% a6 = sqrt(power((p6(2)+ pc6(2))/2 - t(2),2) + power((p6(3) + pc6(3))/2 - t(3),2));
% 
% b1 = sqrt(power((p1(1)+ pc1(1))/2 - t(1),2) + power((p1(3) + pc1(3))/2 - t(3),2));
% b2 = sqrt(power((p2(1)+ pc2(1))/2 - t(1),2) + power((p2(3) + pc2(3))/2 - t(3),2));
% b3 = sqrt(power((p3(1)+ pc3(1))/2 - t(1),2) + power((p3(3) + pc3(3))/2 - t(3),2));
% b4 = sqrt(power((p4(1)+ pc4(1))/2 - t(1),2) + power((p4(3) + pc4(3))/2 - t(3),2));
% b5 = sqrt(power((p5(1)+ pc5(1))/2 - t(1),2) + power((p5(3) + pc5(3))/2 - t(3),2));
% b6 = sqrt(power((p6(1)+ pc6(1))/2 - t(1),2) + power((p6(3) + pc6(3))/2 - t(3),2));
% 
% 
% c1 = sqrt(power((p1(2)+ pc1(2))/2 - t(2),2) + power((p1(1) + pc1(1))/2 - t(1),2));
% c2 = sqrt(power((p2(2)+ pc2(2))/2 - t(2),2) + power((p2(1) + pc2(1))/2 - t(1),2));
% c3 = sqrt(power((p3(2)+ pc3(2))/2 - t(2),2) + power((p3(1) + pc3(1))/2 - t(1),2));
% c4 = sqrt(power((p4(2)+ pc4(2))/2 - t(2),2) + power((p4(1) + pc4(1))/2 - t(1),2));
% c5 = sqrt(power((p5(2)+ pc5(2))/2 - t(2),2) + power((p5(1) + pc5(1))/2 - t(1),2));
% c6 = sqrt(power((p6(2)+ pc6(2))/2 - t(2),2) + power((p6(1) + pc6(1))/2 - t(1),2));

A1 = (p4 - t)/( norm(p4 - t) );
A2 = (p2 - t)/( norm(p2 - t) );
A3 = (p6 - t)/( norm(p6 - t) );

B1 = cross(A1,f1x);
B2 = cross(A1,f1y);
B3 = cross(A1,f1z);
B4 = cross(A2,f1a);
B5 = cross(A2,f1b);
B6 = cross(A3,f1t);


% B1(1) + B2(1) + B3(1) + B4(1) + B5(1)+ B6(1)= 0 ;
% B1(2) + B2(2) + B3(2) + B4(2) + B5(2) + B6(2)= 0 ;
% B1(3) + B2(3) + B3(3) + B4(3) + B5(3) + B6(3)= 0 ;
% 

coeff_matrix = [
     f1x(1)  f1y(1)  f1z(1)  f1a(1)  f1b(1)  f1t(1);
     f1x(2)  f1y(2)  f1z(2)  f1a(2)  f1b(2)  f1t(2);
     f1x(3)  f1y(3)  f1z(3)  f1a(3)  f1b(3)  f1t(3)
     B1(1)   B2(1)   B3(1)   B4(1)   B5(1)  B6(1);
     B1(2)   B2(2)   B3(2)   B4(2)   B5(2)  B6(2);
     B1(3)   B2(3)   B3(3)   B4(3)   B5(3)  B6(3);
       ];
%    mat_x = [ F1; F2; F3; F4; F5; F6; ];
for i = 1:5

 F = [Fx(i); Fy(i); Fz(i) ; 0; 0 ; 0;];
     X(i,:) = pinv(coeff_matrix)*F;
      
end

theta1 = acos(dot(vec1,f1a));
theta2 = acos(dot(vec2,f1a));

 mat2 = [cos(theta1) cos(theta2);
        -sin(theta1) sin(theta2);
        ];
for i = 1:5
    F_l = [ X(i,4); 
            X(i,5);
          ];
    lower_f(i,:) = F_l * pinv(mat2);
end    
 mat3 = [cos(vec3(1)) cos(vec4(1)) cos(vec5(1));
         cos(vec3(2)) cos(vec4(2)) cos(vec5(2));
         cos(vec3(3)) cos(vec4(3)) cos(vec5(3));
         ];
 for i = 1:5    
 F_u = [ X(i,1); 
         X(i,2);
         X(i,3);
         ];
    upper_f(i,:) = F_u * pinv(mat3);
 end    
 %torque about upper arm knuckle point
 % point = [166.355,52.41, 24.906]
 %%% tie rod * pt-> tie rod pt + lower * upper knuckle-> lower knuckle +
 %%% pull rod * pt-> pull rod = zero
 d1 = (p6- p5)/norm(p6 - p5);
 d2 = (p1- p5)/norm(p1 - p5);
 d3 = (p3- p5)/norm(p3 - p5);
 
 
 cross(d1 ,X(i,6)) + cross(d2, X(i,5)) + cross(d2, X(i,4)) + Required cross(d3,vec3)= 
    
    
    
    
    

