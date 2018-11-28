function [A,B,nominal_x,nominal_input] = getLinearSys_old(nominal_x0,nominal_data_setting)

syms Z u w Fx Fz
syms t

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
wdot = (1/(m + MB33_plue_DetMB33))*(Fz + 0.5*dDetMB11_over_dZ*u^2 - 0.5*(dDetMB31_over_dZ-dDetMB13_over_dZ)*u*w - 0.5*dDetMB33_over_dZ*w^2 - (m+MB31_plus_DetMB31)*udot);

%%%%%%% Compute nominal data
% We have three system-dependent states, 5 variables to set to get the
% equilibrim (or any other situation we want) motion, so we need to set 2
% parameters to get the full rank system
Xdot_sub = subs(Xdot, [Z u], [nominal_data_setting(1,1),nominal_data_setting(2,1)]);
Zdot_sub = subs(Zdot, [Z u], [nominal_data_setting(1,1),nominal_data_setting(2,1)]);
udot_sub = subs(udot, [Z u], [nominal_data_setting(1,1),nominal_data_setting(2,1)]);
wdot_sub = subs(wdot, [Z u], [nominal_data_setting(1,1),nominal_data_setting(2,1)]);
eqns = [Zdot_sub == 0, udot_sub == 0, wdot_sub == 0];
vars = [w Fx Fz];
sol = solve(eqns, vars);
nominal_input = [sol.Fx; sol.Fz];
nominal_x = [nominal_x0(1,1)+Xdot_sub*t; nominal_data_setting(1,1); nominal_data_setting(2,1); sol.w];
nominal_x_dependent = [nominal_data_setting(1,1); nominal_data_setting(2,1); sol.w];

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

A = [subs(df1dX, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df1dZ, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df1du, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df1dw, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']);...
     subs(df2dX, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df2dZ, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df2du, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df2dw, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']);...
     subs(df3dX, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df3dZ, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df3du, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df3dw, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']);...
     subs(df4dX, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df4dZ, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df4du, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df4dw, [Z u w Fx Fz], [nominal_x_dependent' nominal_input'])];
B = [subs(df1dFx, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df1dFz, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']);...
     subs(df2dFx, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df2dFz, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']);...
     subs(df3dFx, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df3dFz, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']);...
     subs(df4dFx, [Z u w Fx Fz], [nominal_x_dependent' nominal_input']), subs(df4dFz, [Z u w Fx Fz], [nominal_x_dependent' nominal_input'])];
