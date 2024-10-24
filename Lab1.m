pkg load control
clear all
close all

m = 1;
l = 0.1;
Jy = (1/12) * m*(l^2 + l^2) %[kg m^2]
bz = 22703.6e-9;

                    %%% 1 - Cubesat pitch dynamics
a = bz / Jy
s = tf('s');
Gs = a / s^2

figure(1);
bode(Gs);
%margin(Gs);

figure(2);
pzmap(Gs);

figure(3);
step(Gs);

poles_Gs=pole(Gs)


                    %%% 2 - PID Control
                    % P
kt=1;
Ks1 = kt / a;
Ls1 = kt / s^2;
Ts1_1 = 1 / (s^2 + 1);
Ts1_2 = 2 / (s^2 + 2);
Ts1_3 = 3 / (s^2 + 3);

figure(4);
rlocus(Ls1);

figure(5);
step(Ts1_1, Ts1_2, Ts1_3);


                    % PD
kw=1;
Ks2 = (kw*s) / a;
Ls2 = kw / s;
Ts2_1 = 1 / (s + 1);
Ts2_2 = 2 / (s + 2);
Ts2_3 = 3 / (s + 3);

figure(6);
rlocus(Ls2);

figure(7);
step(Ts2_1, Ts2_2, Ts2_3);


                    % PID
Ti=41;
Td=10;

CPIDs = 1*(1 + (1/(Ti*s)) + (s*Td));
Ls3 = Gs * CPIDs

figure(8);
pzmap(Ls3);

figure(9);
rlocus(Ls3);

kt=15;
Ts3 = minreal((kt*Ls3) / (1+(kt*Ls3))) %simplifica a expressão, cortando os ss

figure(10);
step(Ts3);

figure(11);
margin(Ls3)
