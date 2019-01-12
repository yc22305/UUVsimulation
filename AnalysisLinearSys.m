clear;
cmap = colormap(hsv); close;
cmap1 = colormap(spring); close;
cmap2 = colormap(summer); close;
cmap3 = colormap(winter); close;
N_states = 4;
N_inputs = 2;

%% Fixed u. Varying Z
nominal_u = 5;
N = 100;
Zk_start = 0.5; Zk_final = 5;
Zk = linspace(Zk_start,Zk_final,N);

fhandle = figure;
color_idx = 1;
for i = 1:N
    nominal_x0 = [0; Zk(i); nominal_u; 0]; % matters to X
    nominal_data_setting = [Zk(i); nominal_u]; % Set Z, u.
    [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
    r = eig(A(2:end,2:end));
    
    if (color_idx > size(cmap,1))
        color_idx = 1;
    end
    plot3(real(r),imag(r),Zk(i)*ones(size(r,1),1),'Color',cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
    color_idx = color_idx+1;
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying Zk: ', num2str(Zk_start), ' to ', num2str(Zk_final)],['u = ', num2str(nominal_u)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);

%% Fixed u. Varying Z. (see the change with three different u)
nominal_u = [2 5 7];
% cmaps = {cmap1, cmap2, cmap3};
N = 100;
Zk_start = 0.5; Zk_final = 5;
Zk = linspace(Zk_start,Zk_final,N);
p_real = zeros(3,N);
p_imag = zeros(3,N);

fhandle = figure;
for j = 1:3
%     current_cmap = cmaps{j};
%     color_idx = 1;
    for i = 1:N
        nominal_x0 = [0; Zk(i); nominal_u(j); 0]; % matters to X
        nominal_data_setting = [Zk(i); nominal_u(j)]; % Set Z, u.
        [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
        r = eig(A(2:end,2:end));
        
%         if (color_idx > size(current_cmap,1))
%             color_idx = 1;
%         end
%         plot3(real(r),imag(r),Zk(i)*ones(size(r,1),1),'Color',current_cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
%         color_idx = color_idx+1;
        switch j
            case 1 
                plot3(real(r),imag(r),Zk(i)*ones(size(r,1),1),'b','Marker','.','MarkerSize',8,'LineStyle','none'), hold on;
            case 2
                plot3(real(r),imag(r),Zk(i)*ones(size(r,1),1),'r','Marker','.','MarkerSize',8,'LineStyle','none'), hold on;
            case 3 
                plot3(real(r),imag(r),Zk(i)*ones(size(r,1),1),'g','Marker','.','MarkerSize',8,'LineStyle','none'), hold on;
        end
        conjidx = r(imag(r)~=0);
        p_real(j,i) = real(conjidx(1)); % pick one eigenvalue of the conjugate pair
        p_imag(j,i) = imag(conjidx(1)); % pick one eigenvalue of the conjugate pair
    end
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying Z: ', num2str(Zk_start), ' to ', num2str(Zk_final)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);
figure,
plot(Zk,p_real(1,:),'LineWidth', 2); hold on,
plot(Zk,p_real(2,:),'r','LineWidth', 2); hold on,
plot(Zk,p_real(3,:),'g','LineWidth', 2);
title('Real part of the conjugate eigenvalues (fixed u)'), xlabel('depth (Z)') ,ylabel('Real part');
legend(['u = ',num2str(nominal_u(1))],['u = ',num2str(nominal_u(2))],['u = ',num2str(nominal_u(3))]);
figure,
plot(Zk,p_imag(1,:),'LineWidth', 2); hold on,
plot(Zk,p_imag(2,:),'r','LineWidth', 2); hold on,
plot(Zk,p_imag(3,:),'g','LineWidth', 2);
title('Imaginary part of the conjugate eigenvalues (fixed u)'), xlabel('depth (Z)') ,ylabel('Imaginary part');
legend(['u = ',num2str(nominal_u(1))],['u = ',num2str(nominal_u(2))],['u = ',num2str(nominal_u(3))]);

%% Fixed Z. Varying u
nominal_Z = 0.5;
N = 100;
uk_start = 1; uk_final = 10;
uk = linspace(uk_start,uk_final,N);

fhandle = figure;
color_idx = 1;
for i = 1:N
    nominal_x0 = [0; nominal_Z; uk(i); 0]; % matters to X
    nominal_data_setting = [nominal_Z; uk(i)]; % Set Z, u.
    [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
    r = eig(A(2:end,2:end));
    
    if (color_idx > size(cmap,1))
        color_idx = 1;
    end
    plot3(real(r),imag(r),uk(i)*ones(size(r,1),1),'Color',cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
    color_idx = color_idx+1;
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying u: ', num2str(uk_start), ' to ', num2str(uk_final)],['Z = ', num2str(nominal_Z)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);

%% Fixed Z. Varying u. (see the change with three different Z)
nominal_Z = [0.5 1.5 2.5];
% cmaps = {cmap1, cmap2, cmap3};
N = 100;
uk_start = 1; uk_final = 10;
uk = linspace(uk_start,uk_final,N);
p_real = zeros(3,N);

fhandle = figure;
for j = 1:3
%     current_cmap = cmaps{j};
%     color_idx = 1;
    for i = 1:N
        nominal_x0 = [0; nominal_Z(j); uk(i); 0]; % matters to X
        nominal_data_setting = [nominal_Z(j); uk(i)]; % Set Z, u.
        [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
        r = eig(A(2:end,2:end));
        
%         if (color_idx > size(current_cmap,1))
%             color_idx = 1;
%         end
%         plot3(real(r),imag(r),uk(i)*ones(size(r,1),1),'Color',current_cmap(color_idx,:),'Marker','.','LineStyle','none'), hold on;
%         color_idx = color_idx+1;
        switch j
            case 1 
                plot3(real(r),imag(r),uk(i)*ones(size(r,1),1),'b','Marker','.','MarkerSize',8,'LineStyle','none'), hold on;
            case 2
                plot3(real(r),imag(r),uk(i)*ones(size(r,1),1),'r','Marker','.','MarkerSize',8,'LineStyle','none'), hold on;
            case 3 
                plot3(real(r),imag(r),uk(i)*ones(size(r,1),1),'g','Marker','.','MarkerSize',8,'LineStyle','none'), hold on;
        end
        conjidx = r(imag(r)~=0);
        p_real(j,i) = real(conjidx(1)); % pick one eigenvalue of the conjugate pair
        p_imag(j,i) = imag(conjidx(1)); % pick one eigenvalue of the conjugate pair
    end
end
view(0,90);
title({'Poles of linearized open-loop system about the equilibrium state',['Varying u: ', num2str(uk_start), ' to ', num2str(uk_final)]});
dcm_obj = datacursormode(fhandle);
set(dcm_obj,'UpdateFcn',@myCursor);
figure,
plot(uk,p_real(1,:),'LineWidth', 2); hold on,
plot(uk,p_real(2,:),'r','LineWidth', 2); hold on,
plot(uk,p_real(3,:),'g','LineWidth', 2);
title('Real part of the conjugate eigenvalues (fixed Z)'), xlabel('surge speed (u)') ,ylabel('Real part');
legend(['Z = ',num2str(nominal_Z(1))],['Z = ',num2str(nominal_Z(2))],['Z = ',num2str(nominal_Z(3))]);
figure,
plot(uk,p_imag(1,:),'LineWidth', 2); hold on,
plot(uk,p_imag(2,:),'r','LineWidth', 2); hold on,
plot(uk,p_imag(3,:),'g','LineWidth', 2);
title('Imaginary part of the conjugate eigenvalues (fixed Z)'), xlabel('surge speed (u)') ,ylabel('Imaginary part');
legend(['Z = ',num2str(nominal_Z(1))],['Z = ',num2str(nominal_Z(2))],['Z = ',num2str(nominal_Z(3))]);

%% PID controller design (Classical controller design--focus on SISO)
clear; clc;
N_states = 4;

% A classical control law suggests that by observing a transfer function,
% one could tune parameters to adjust the performance of a system. 
% However, some systems are sufficiently complicated that the classical
% control strategy fails, and even a "general" design method is not proposed.
% An example is a MIMO system with non-diagonal transfer function matrix.
% More modern methods might be

% First analytically compute the linearized system, and then find the
% transfer function matrix as a function of depth and surge speed
% Note that we only consider the surge speed and depth, so the transfer function
% matrix is 2-by-2 (input: Fu, Fw / output: Z, u)
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
D_coe = MB31_plus_DetMB31;
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
G11b = [h e*g]; % numerator
G12b = [i 0];
G21b = [g -(g*f-c*h) -g*d];
G22b = [c*i 0];

% %%%%%%%%% Plot of each mode's weight %%%%%%%%%
% % Partial fraction of the transfer functions indicates the dominant mode of each output, which might help us establish the control design strategy. 
% % Here we consider two types of modes: first-order and second-order mode.
% % Note that for our system, a pole is always zero (e.g. Type-1 system), independent of depth and surge speed; this implies that a integration controller is not necessary.
% % Note that kG11,... are always empty because the system is physically realizable.
% nominal_u_setting = 5;
% nominal_Z_setting = 0.2:0.02:5;
% N =length(nominal_Z_setting);
% for idx = 1:N
%     nominal_x0 = [0; nominal_Z_setting(idx); nominal_u_setting; 0]; % nominal initial state: X, Z, u, w
%     nominal_data_setting = [nominal_Z_setting(idx); nominal_u_setting]; % Set Z, u.
%     [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
%     
%     a_subs = double(subs(a,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G11b_subs = double(subs(G11b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G12b_subs = double(subs(G12b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G21b_subs = double(subs(G21b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G22b_subs = double(subs(G22b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     [rG11,pG11,kG11] = residue(G11b_subs,a_subs);
%     [rG12,pG12,kG12] = residue(G12b_subs,a_subs);
%     [rG21,pG21,kG21] = residue(G21b_subs,a_subs);
%     [rG22,pG22,kG22] = residue(G22b_subs,a_subs);
%     
%     figure(1), % weight
%     h11 = plot(nominal_Z_setting(idx),2*abs(rG11(imag(rG11)~=0)),'b.'); hold on,
%     h12 = plot(nominal_Z_setting(idx),abs(rG11(imag(rG11)==0)),'r.'); hold on,
%     figure(2),
%     h21 = plot(nominal_Z_setting(idx),2*abs(rG12(imag(rG12)~=0)),'b.'); hold on,
%     h22 = plot(nominal_Z_setting(idx),abs(rG12(imag(rG12)==0)),'r.'); hold on,
%     figure(3),
%     h31 = plot(nominal_Z_setting(idx),2*abs(rG21(imag(rG21)~=0)),'b.'); hold on,
%     h32 = plot(nominal_Z_setting(idx),abs(rG21(imag(rG21)==0)),'r.'); hold on,
%     figure(4),
%     h41 = plot(nominal_Z_setting(idx),2*abs(rG22(imag(rG22)~=0)),'b.'); hold on,
%     h42 = plot(nominal_Z_setting(idx),abs(rG22(imag(rG22)==0)),'r.'); hold on,
% end
% figure(1), title('G11 Weight'), ylabel('Weight'), xlabel('Z'), legend([h11(1) h12(1)],'second-order term','first-order term');
% figure(2), title('G12 Weight'), ylabel('Weight'), xlabel('Z'), legend([h21(1) h22(1)],'second-order term','first-order term');
% figure(3), title('G21 Weight'), ylabel('Weight'), xlabel('Z'), legend([h31(1) h32(1)],'second-order term','first-order term');
% figure(4), title('G22 Weight'), ylabel('Weight'), xlabel('Z'), legend([h41(1) h42(1)],'second-order term','first-order term');
% 
% nominal_Z_setting = 0.5;
% nominal_u_setting = 2:0.01:8;
% N =length(nominal_u_setting);
% for idx = 1:N
%     nominal_x0 = [0; nominal_Z_setting; nominal_u_setting(idx); 0]; % nominal initial state: X, Z, u, w
%     nominal_data_setting = [nominal_Z_setting; nominal_u_setting(idx)]; % Set Z, u.
%     [A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
%     
%     a_subs = double(subs(a,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G11b_subs = double(subs(G11b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G12b_subs = double(subs(G12b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G21b_subs = double(subs(G21b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     G22b_subs = double(subs(G22b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
%     [rG11,pG11,kG11] = residue(G11b_subs,a_subs);
%     [rG12,pG12,kG12] = residue(G12b_subs,a_subs);
%     [rG21,pG21,kG21] = residue(G21b_subs,a_subs);
%     [rG22,pG22,kG22] = residue(G22b_subs,a_subs);
% 
%     figure(5), % weight
%     h61 = plot(nominal_u_setting(idx),2*abs(rG11(imag(rG11)~=0)),'b.'); hold on,
%     h62 = plot(nominal_u_setting(idx),abs(rG11(imag(rG11)==0)),'r.'); hold on,
%     figure(6),
%     h71 = plot(nominal_u_setting(idx),2*abs(rG12(imag(rG12)~=0)),'b.'); hold on,
%     h72 = plot(nominal_u_setting(idx),abs(rG12(imag(rG12)==0)),'r.'); hold on,
%     figure(7),
%     h81 = plot(nominal_u_setting(idx),2*abs(rG21(imag(rG21)~=0)),'b.'); hold on,
%     h82 = plot(nominal_u_setting(idx),abs(rG21(imag(rG21)==0)),'r.'); hold on,
%     figure(8),
%     h91 = plot(nominal_u_setting(idx),2*abs(rG22(imag(rG22)~=0)),'b.'); hold on,
%     h92 = plot(nominal_u_setting(idx),abs(rG22(imag(rG22)==0)),'r.'); hold on,
% end
% figure(5), title('G11 Weight'), ylabel('Weight'), xlabel('u'), legend([h61(1) h62(1)],'second-order term','first-order term');
% figure(6), title('G12 Weight'), ylabel('Weight'), xlabel('u'), legend([h71(1) h72(1)],'second-order term','first-order term');
% figure(7), title('G21 Weight'), ylabel('Weight'), xlabel('u'), legend([h81(1) h82(1)],'second-order term','first-order term');
% figure(8), title('G22 Weight'), ylabel('Weight'), xlabel('u'), legend([h91(1) h92(1)],'second-order term','first-order term');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%% Specific nominal states %%%%%%%%%
% From the plot of modes' weight, valid nominal states are chosen to design
% the controller.
nominal_data_setting = [0.5; 2]; % Set Z, u.
nominal_x0 = [0; nominal_data_setting(1); nominal_data_setting(2); 0]; % nominal initial state: X, Z, u, w
[A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);

a_subs = double(subs(a,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
G11b_subs = double(subs(G11b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
G12b_subs = double(subs(G12b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
G21b_subs = double(subs(G21b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
G22b_subs = double(subs(G22b,[Z u Fw], [nominal_x(2) nominal_x(3) nominal_input(2)]));
[rG11,pG11,kG11] = residue(G11b_subs,a_subs)
[rG12,pG12,kG12] = residue(G12b_subs,a_subs)
[rG21,pG21,kG21] = residue(G21b_subs,a_subs)
[rG22,pG22,kG22] = residue(G22b_subs,a_subs)

%%%%%%%%% Proportional gain (P) design applied to the surge force%%%%%%%%%
% This design aims to set satisfactory convergence time for the surge speed,
% which is mainly controlled by the surge force.
% Only P controller is needed because G21 can be regarded as first-order
% Note that the surge force will also influence other states becasue of
% the existence of coupling transfer functions
G21timeConst = 5;
for idx = 1:N_states-1 % Find all the first-order term with a real eigenvalue to calculate all the possible candidate Kp 
    conjidx = find(pG21==conj(pG21(idx)));
    if conjidx == idx
        Kpu = (1/G21timeConst-(-pG21(idx)))/rG21(idx);
        break;
    end
end
G21fbsys = feedback(Kpu*tf(G21b_subs,a_subs),1);
[G21fb_b,G21fb_a] = tfdata(G21fbsys,'v'); % To see the property of the SISO feedback system
[rG21fb,pG21fb,kG21fb] = residue(G21fb_b,G21fb_a)
%%%%%%%%% Proportional derivative gains (PD) design applied to the surge force %%%%%%%%%
% This design aims to set satisfactory convergence time for the depth,
% which is mainly controlled by the heave force.
% PD controller is needed because G12 is second-order, which requires 2
% parameters to give total authorization
% Note that the heave force will also influence other states becasue of
% the existence of coupling transfer functions
G12dampingRatio = sin(40/180*pi);
G12sigma = 0.5; 
G12naturalFrequency = G12sigma/G12dampingRatio;
syms Kpww Kdww
for idx = 1:N_states-1
    conjidx = find(pG21==conj(pG12(idx)));
    if conjidx ~= idx
        G12_sin_residue = 2*abs(imag(rG12(idx)))*abs(imag(pG12(idx))) ; % the residue of the zero-order part
        eqns = [(-pG12(idx))*(-pG12(conjidx))+Kpww*G12_sin_residue == G12naturalFrequency^2, ...
                (-pG12(idx))+(-pG12(conjidx))+Kdww*G12_sin_residue == 2*G12sigma];
        gains = solve(eqns,[Kpww Kdww]);
        Kpw = double(gains.Kpww);
        Kdw = double(gains.Kdww);
        break;
    end
end
G12fbsys = feedback((Kpw+Kdw*tf('s'))*tf(G12b_subs,a_subs),1);
[G12fb_b,G12fb_a] = tfdata(G12fbsys,'v'); % To see the property of the SISO feedback system
[rG12fb,pG12fb,kG12fb] = residue(G12fb_b,G12fb_a)

% control gain matrix
K = [0 Kpu 0; 0 0 0]
% Check eigenvalues
lambda = eig(A(2:end,2:end)-B(2:end,:)*K)

%% Pole placement
nominal_data_setting = [0.5; 2]; % Set Z, u.
nominal_x0 = [0; nominal_data_setting(1); nominal_data_setting(2); 0]; % nominal initial state: X, Z, u, w
[A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);

dampingRatio = sin(40/180*pi);
sigma = 0.5; 
naturalFrequency = sigma/dampingRatio;
Frequency = sqrt(1-dampingRatio^2)*naturalFrequency;
timeConst = 5;
p = [-1/timeConst; -sigma+1i*Frequency; -sigma-1i*Frequency];
K = place(A(2:end,2:end),B(2:end,:),p)
[eigVector,lambda] = eig(A(2:end,2:end)-B(2:end,:)*K)

%% LQR
nominal_data_setting = [0.5; 2]; % Set Z, u.
nominal_x0 = [0; nominal_data_setting(1); nominal_data_setting(2); 0]; % nominal initial state: X, Z, u, w
[A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);

DevPercentage = 10;
x0 = nominal_x0*(1+0.01*DevPercentage); % initial state: X, Z, u, w

% values used to non-dimensionalize Q, R and states and inputs, depending on the overshoot which is set to limited in 10%
Zbound = abs(nominal_x(2)-x0(2))*1.1; 
ubound = abs(nominal_x(3)-x0(3))*1.1;

Q = eye(3)*10;
Q(1,1) = Q(1,1)/Zbound^2; Q(3,3) = Q(3,3)/Zbound^2;
Q(2,2) = Q(2,2)/ubound^2;
R = eye(2);

[K,S,lambda] = lqr(A(2:end,2:end),B(2:end,:),Q,R)