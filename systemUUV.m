function dxdt = systemUUV(tt,x,nominal_x,nominal_input,KpGains,KdGains)

global x_errLast ttLast FuLast FwLast
syms t

nominal_x = double(subs(nominal_x,t,tt));
nominal_input = double(subs(nominal_input,t,tt));

%%% states of the UUV
m = 116.355; % mass of the UUV
Z = x(2,1);
u = x(3,1);
w = x(4,1);
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

% control forces
x_err = nominal_x-x;
dtt = tt-ttLast;
if dtt>0
    Fu = nominal_input(1,1) + (KpGains(1,:)*x_err + KdGains(1,:)*(x_err-x_errLast)/dtt); % Consider the feedback errors contributed by all the components in xdir
    Fw = nominal_input(2,1) + (KpGains(2,:)*x_err + KdGains(2,:)*(x_err-x_errLast)/dtt); % Consider the feedback errors contributed by all the components in zdir
    FuLast = Fu;
    FwLast = Fw;
    x_errLast = x_err;
    ttLast = tt;
else
    Fu = FuLast;
    Fw = FwLast;
end

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

dxdt = [Xdot; Zdot; udot; wdot];

