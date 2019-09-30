function [X] = rear_force_distriution_in_connected_links()
%forces as compression are positive
%%% look as points in diagram
% 6 points on upright as input as a 1*3 matrix [p1x p1y p1z]
pc1 = [190.5,  13.97, 33.02]; 
pc2 = [190.5,  13.97, 45.72]; 
pc3 = [180.34, 33.02, 33.02];
pc4 = [128.270, 33.02, 33.020];
pc5 = [128.270, 33.02, 40.640];
pc6 = [128.070, 41.550,67.460];
% 5 points on chasis as input as a 1*3 matrix 
p1 = [180.34 ,53.342, 21.59]; % lower suspension link 1
p2 = [180.34 ,53.342, 21.59]; % lower suspension link 2
p3 = [170.10 ,29.24, 21.59]; % push rod
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

f1x = [1,0,0]; % forces at lower a arm junction point
f1y = [0,1,0];
f1z = [0,0,1];

f1a = (vec4 + vec5)/(norm(vec4 + vec5));
f1b = (vec4 - vec5)/(norm(vec4 - vec5));

f1t = vec6;

A1 = (p4 - t);%upper junction
A2 = (p2 - t);%lower junction
A3 = (p6 - t);% tie rod

B1 = cross(A2,f1x);
B2 = cross(A2,f1y);
B3 = cross(A2,f1z);
B4 = cross(A1,f1a);
B5 = cross(A1,f1b);
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

theta1 = acos(dot(vec4,f1a));
theta2 = acos(dot(vec5,f1a));
%%% f1u*cos(theta1) + f2u*cos(theta2) = f1a
%%% -f1u*sin(theta1) + f2u*sin(theta2) = f1b
 mat2 = [cos(theta1) -sin(theta1);
         cos(theta2)  sin(theta2);
        ];
for i = 1:5
    F_u = [ X(i,4)   X(i,5);
          ];
    upper_f(i,:) = F_u * pinv(mat2);
 end  
%%% took all vectors component along difinite axis 
%f1x will remain same.. 
g1 = [100 200 50];
g2 = cross(vec1,vec2)/ norm(cross(vec1,vec2));
for i = 1:5 
theta(i,:) =  acos((X(i,2)*g1(2))/(norm(X(i,2)) * norm(g1)));
end
% 
% mat3 = [vec3(1) vec4(1) vec5(1); % component along x axis
%          vec3(2) vec4(2) vec5(2); % component along y axis
%          vec3(3) vec4(3) vec5(3); % component along z axis
%          ];
% y and z components are being rotated to get in the plane of upper link a
% arm
for i = 1:5
mat3 = [ 0  X(i,2)  X(i,3);
        ];
rot_mat = [1         0            0;
           0  cos(theta(i)) sin(theta(i));
           0 -sin(theta(i)) cos(theta(i));
           ];
       
new_forces_value(i,:) =  mat3 * rot_mat ;

upv = (pc1 - pc2)/norm(pc1 - pc2) ; %upperlinkvector
k1 = (p3 - pc1)/ norm(p3 - pc1); %pull rod force point of application to torque point3

pushrod_force(i,:) = -(new_forces_value(i,3)*dot(upv,cross(vec1,g2),3))/ dot(upv,cross(k1,vec3),3); % magnitude only
%%% torque balancing about the upv vector

end

theta3 = acos(dot(vec1,f1x));
theta4 = acos(dot(vec2,f1x));

mat4 = [cos(theta3)  -sin(theta3);
        cos(theta4)   sin(theta4);
        ];

for i = 1:5
    F_l = [ X(i,1)   new_forces_value(i,2);
          ];
    lower_f(i,:) = F_l * pinv(mat4);
end  
 disp('upper_f is')
  disp(upper_f)
   disp('lower_f is')
    disp(lower_f)
     disp('tie rod f is')
      disp(X(i,6))
       disp('pullrod_force')
        disp(pushrod_force)


    
    
    

