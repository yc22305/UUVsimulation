% This function is used to get DC gain and controller parameters
% K is a 3-by-1 vector which is composed by Kp, Ki, and Kd
% All the desired parameters are set as follows:
% First-order system:
%   time constant: 10 s
% Second-order system
%   overshoot <= 0.2
%   rising time <= 5
%   settling time <= 10
%   ---we choose:
%           damping ratio: sin(40/180*pi)
%           sigma: 0.5

function K = getPID(order,nominal_x,nominal_input)

N_states = 4;
syms Z u Fw % the nominal data which are not fixed

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
% expressed as coefficients
A_coe = m+MB11_plus_DetMB11;
B_coe = dDetMB11_over_dZ;
C_coe = dDetMB13_over_dZ;
D_coe = m+MB31_plus_DetMB31;
E_coe = m+MB33_plue_DetMB33;
F_coe = -0.5*dDetMB11_over_dZ;
G_coe = 0.5*(dDetMB31_over_dZ-dDetMB13_over_dZ);
H_coe = 0.5*dDetMB33_over_dZ;

% elements of matrices of the linearized system
c = -B_coe/A_coe*u;
d = -E_coe^(-2)*diff(E_coe,Z)*(Fw-F_coe*u^2)-1/E_coe*diff(F_coe,Z)*u^2;
e = -F_coe/E_coe*2*u;
f = u/E_coe*(D_coe*B_coe/A_coe-G_coe);
g = 1/A_coe;
h = -D_coe/(A_coe*E_coe);
i = 1/E_coe;

% numerator and denomiator of each transfer function.
% Note that each transfer function has the same poles
a = [1 -f -(d+c*e) 0]; % denomiator
G12b = [i 0];
G21b = [g -(g*f-c*h) -g*d];

a_subs = double(subs(a,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
G12b_subs = double(subs(G12b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
G21b_subs = double(subs(G21b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
[rG12,pG12,kG12] = residue(G12b_subs,a_subs);
[rG21,pG21,kG21] = residue(G21b_subs,a_subs);

K = zeros(3,1);
if order == 1 % first-order system
    G21timeConst = 5;
    for idx = 1:N_states-1 % Find all the first-order term with a real eigenvalue to calculate all the possible candidate Kp
        conjidx = find(pG21==conj(pG21(idx)));
        if conjidx == idx
            K(1) = (1/G21timeConst-(-pG21(idx)))/rG21(idx);
            break;
        end
    end
else % second-order system
    G12dampingRatio = sin(40/180*pi);
    G12sigma = 0.5; % to calculate the natural frequency
    G12naturalFrequency = G12sigma/G12dampingRatio;
    syms Kpww Kdww
    for idx = 1:N_states-1
        conjidx = find(pG12==conj(pG12(idx)));
        if conjidx ~= idx
            G12_sin_residue = 2*abs(imag(rG12(idx)))*abs(imag(pG12(idx))); % the residue of the zero-order part
            eqns = [(-pG12(idx))*(-pG12(conjidx))+Kpww*G12_sin_residue == G12naturalFrequency^2, ...
                    (-pG12(idx))+(-pG12(conjidx))+Kdww*G12_sin_residue == 2*G12sigma];
            gains = solve(eqns,[Kpww Kdww]);
            K(1) = double(gains.Kpww);
            K(3) = double(gains.Kdww);
            break;
        end
    end
end