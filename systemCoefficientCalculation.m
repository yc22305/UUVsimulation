clear;
syms Z u w Fu Fw
%% Decouple udot and wdot
%%% parameters of the UUV
m = 116.355; % mass of the UUV
% Added mass. Submerged added mass and surfave perturbed added mass. These
% formulas are got from previous experiments.
MB11_plus_DetMB11 = 11.17*Z^5 - 41.93*Z^4 + 63.92*Z^3 -50.67*Z^2 + 21.85*Z + 0.8278;
MB31_plus_DetMB31 = -0.2532*Z^7 + 1.159*Z^6 - 2.221*Z^5 + 2.307*Z^4 - 1.339*Z^3 + 0.4943*Z^2 - 0.09445*Z + 0.00741;
MB33_plue_DetMB33 = 448.7*Z^5 - 1608*Z^4 + 2290*Z^3 -1636*Z^2 + 600.9*Z + 11.42;
% Added mass gradient. Relative to damping matrix.
dDetMB11_over_dZ = -67.55*Z^5 + 260.4*Z^4 - 405.3*Z^3 + 323.4*Z^2 -136*Z + 25.31;
dDetMB31_over_dZ = -0.7414*Z^6 + 3.317*Z^5 - 5.438*Z^4 + 4.937*Z^3 - 2.475*Z^2 + 0.6497*Z - 0.0703; 
dDetMB13_over_dZ = 0.05967*Z^5 - 0.2144*Z^4 + 0.3022*Z^3 - 0.2087*Z^2 + 0.07117*Z - 0.01007;
dDetMB33_over_dZ = -3157*Z^5 + 11770*Z^4 - 17430*Z^3 + 12920*Z^2 - 4853*Z + 757.1;
% Calculate all the needed coefficients to solve coupling udot and wdot
A_coe = m+MB11_plus_DetMB11;
B_coe = dDetMB11_over_dZ;
C_coe = dDetMB13_over_dZ;
D_coe = m+MB31_plus_DetMB31;
E_coe = m+MB33_plue_DetMB33;
F_coe = -0.5*dDetMB11_over_dZ;
G_coe = 0.5*(dDetMB31_over_dZ-dDetMB13_over_dZ);
H_coe = 0.5*dDetMB33_over_dZ;
state_udotwdot = [A_coe 0; D_coe E_coe]\[Fu-B_coe*u*w-C_coe*w; Fw-F_coe*u^2-G_coe*u*w-H_coe*w^2];

Xdot = u;
Zdot = w;
udot = state_udotwdot(1);
wdot = state_udotwdot(2);
% coefficients for the rearranged system
[udotCoeffs,udotterms] = coeffs(udot, [Fu Fw u w]);
[wdotCoeffs,wdotterms] = coeffs(wdot, [Fu Fw u w]);
% for udot
AA_coe = udotCoeffs(1); % Fu
BB_coe = udotCoeffs(2); % uw
CC_coe = udotCoeffs(3); % w
% for wdot
DD_coe = wdotCoeffs(1); % Fu
EE_coe = wdotCoeffs(2); % Fw
FF_coe = wdotCoeffs(3); % u^2
GG_coe = wdotCoeffs(4); % uw
HH_coe = wdotCoeffs(5); % w^2
II_coe = wdotCoeffs(6); % w


%% coefficient plots
Zk = [0.5:0.1:3];
figure,
subplot(3,1,1), plot(Zk,subs(AA_coe,Z,Zk)), title('A(z)'), ylabel('Weight'), xlabel('Z');
subplot(3,1,2), plot(Zk,subs(BB_coe,Z,Zk)), title('B(z)'), ylabel('Weight'), xlabel('Z');
subplot(3,1,3), plot(Zk,subs(CC_coe,Z,Zk)), title('C(z)'), ylabel('Weight'), xlabel('Z');
figure,
subplot(3,2,1), plot(Zk,subs(DD_coe,Z,Zk)), title('D(z)'), ylabel('Weight'), xlabel('Z');
subplot(3,2,2), plot(Zk,subs(EE_coe,Z,Zk)), title('E(z)'), ylabel('Weight'), xlabel('Z');
subplot(3,2,3), plot(Zk,subs(FF_coe,Z,Zk)), title('F(z)'), ylabel('Weight'), xlabel('Z');
subplot(3,2,4), plot(Zk,subs(GG_coe,Z,Zk)), title('G(z)'), ylabel('Weight'), xlabel('Z');
subplot(3,2,5), plot(Zk,subs(HH_coe,Z,Zk)), title('H(z)'), ylabel('Weight'), xlabel('Z');
subplot(3,2,6), plot(Zk,subs(II_coe,Z,Zk)), title('I(z)'), ylabel('Weight'), xlabel('Z');
