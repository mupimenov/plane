pkg load control
pkg load signal

Fc = 10;
Tb = 1 / (2 * pi * Fc);
Td = 0.010;
Fd = 1 / Td;

b2 = [1];
a2 = [Tb^2 sqrt(2)*Tb 1];
w2 = tf(b2, a2)
 
[bz2, az2] = impinvar(b2, a2, Fd)
wz2 = filt(bz2, az2, Td)
maz2 = az2(1);
bzc2 = 1.0 * bz2 / maz2;
azc2 = az2 / maz2;
 
bode(wz2);