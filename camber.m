d = linspace(-pi/4,pi/4,301);
ubj = [2.125,57.602,29.2];
lbj = [0.373,59.023,13.97];
a = 300;
SAI = atan((lbj(2)- ubj(2))/(ubj(3) - lbj(3)));
castor = atan( (ubj(1)-lbj(1))/(ubj(3) - lbj(3)) );
rate = zeros(1,a+1);
cambera = zeros(1,a+1);
for i = 1:a
    cambera(i+1) = cambera(i) + (-tan(castor)*cos(d(i)) + tan(SAI)*sin(d(i)))/sqrt(1 + tan(a)^2 + tan(castor)^2);
    rate(i+1) = (-tan(castor)*cos(d(i)) + tan(SAI)*sin(d(i)))/((tan(castor)*sin(d(i)) + tan(a)*cos(d(i)))*tan(cambera(i)) + 1);
end

figure, plot(d*180/pi , cambera*180/pi,'.');
title('Camber vs steering angle')
figure,plot(d*180/pi , rate*180/pi,'.');
title('Camber change rate vs steering angle')
