function [A,B,dxdt] = getLinearSysf()

syms Z u w Fx Fz

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

%%%%%%% independent motion
% f1
Xdot = u;
%%%%%%% dependent motion
% f2
Zdot = w;
% f3
udot = (1/(m + MB11_plus_DetMB11))*(Fx - dDetMB11_over_dZ*u*w - dDetMB13_over_dZ*w^2);
% f4
wdot = (1/(m + MB33_plue_DetMB33))*(Fz + 0.5*dDetMB11_over_dZ*u^2 - 0.5*(dDetMB31_over_dZ-dDetMB13_over_dZ)*u*w - 0.5*dDetMB33_over_dZ*w^2 - MB31_plus_DetMB31*udot);

%%%%%%% derive A and B
% f1
df1dX = 0;
df1dZ = diff(Xdot,Z);
df1du = diff(Xdot,u);
df1dw = diff(Xdot,w);
df1dFx = diff(Xdot,Fx);
df1dFz = diff(Xdot,Fz);
% f2
df2dX = 0;
df2dZ = diff(Zdot,Z);
df2du = diff(Zdot,u);
df2dw = diff(Zdot,w);
df2dFx = diff(Zdot,Fx);
df2dFz = diff(Zdot,Fz);
% f3
df3dX = 0;
df3dZ = diff(udot,Z);
df3du = diff(udot,u);
df3dw = diff(udot,w);
df3dFx = diff(udot,Fx);
df3dFz = diff(udot,Fz);
% f4
df4dX = 0;
df4dZ = diff(wdot,Z);
df4du = diff(wdot,u);
df4dw = diff(wdot,w);
df4dFx = diff(wdot,Fx);
df4dFz = diff(wdot,Fz);

A = [df1dX, df1dZ, df1du, df1dw;...
     df2dX, df2dZ, df2du, df2dw;... 
     df3dX, df3dZ, df3du, df3dw;...
     df4dX, df4dZ, df4du, df4dw];
B = [df1dFx, df1dFz;...
     df2dFx, df2dFz;...
     df3dFx, df3dFz;...
     df4dFx, df4dFz];
dxdt = [Xdot; Zdot; udot; wdot];
