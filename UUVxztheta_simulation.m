clear;
N_states = 4;
N_inputs = 2;
syms t

%%%%%%% nominal data setting (usually equilibrium) and synamics system 
% Note that X doesn't influence the states
% We have 3 system-dependent states, and 5 variables to set to get the
% desired motion. Therefore we need to set 2 parameters to get the rest of
% the steady motion states and inputs
nominal_x0 = [0; 0.5; 2; 0]; % nominal initial state: X, Z, u, w
nominal_data_setting = [0.5; 2]; % Set Z, u. 
[A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);
C = eye(N_states); D = zeros(N_states,N_inputs); % specify C and D matrices for output y
% Also, we can arbitrary nominal data we want to achieve. For example:
% nominal_x = [0+5*t; 0.1; 5; 0];
% nominal_input = [100; 50];

%%%%%%% Gains and initial states
global x_errLast
global ttLast
x0 = [0; 0.6; 1.9; 0]; % initial state: X, Z, u, w
% Control gains
% feedback error
Kp_xdir = [0 0 10 0];
Kp_zdir = [0 5 0 10];
Kd_xdir = [0 0 5 0];
Kd_zdir = [0 5 0 5];
KpGains = [Kp_xdir; Kp_zdir];
KdGains = [Kd_xdir; Kd_zdir];

%%%%%%% ODE numerical solution
tspan = (0:0.5:300)';
%%%%%% ode45 for nonlinear system
x_errLast = nominal_x0-x0;
ttLast = -0.1; % arbitrarily set an initial time
UUVode45 = @(tt,x) systemUUV(tt,x,nominal_x,nominal_input,KpGains,KdGains);
[thist,xhist] = ode15s(UUVode45,tspan,x0);
%%%%%% ode45 for linearized system (output Detx)
UUVode45_Linear = @(tt,Detx) systemUUVLinear(tt,Detx,A,B,KpGains);
[thist_LinearDet,xhist_LinearDet] = ode45(UUVode45_Linear,tspan,x0-nominal_x0);

%%%%%% ode45 for nonlinear system with openloop control
% DetForce_hist = (-feedbackGain*xhist_LinearDet')';
% ControlForce_hist = zeros(length(thist_LinearDet),2);
% ControlForce_hist(:,1) = nominal_input(1,1)+DetForce_hist(:,1);
% ControlForce_hist(:,2) = nominal_input(2,1)+DetForce_hist(:,2);
% UUVode45openLoop = @(tt,x) systemUUVopenLoop(tt,x,ControlForce_hist,thist_LinearDet);
% [thist_open,xhist_open] = ode45(UUVode45openLoop,tspan,x0);


%%%%%% show the nominal trajectory %%%%%
nominal_x_shown = zeros(length(tspan),4);
nominal_x_shown(:,1) = double(subs(nominal_x(1,1),t,tspan)); % Independ
nominal_x_shown(:,2) = double(nominal_x(2,1));
nominal_x_shown(:,3) = double(nominal_x(3,1));
nominal_x_shown(:,4) = double(nominal_x(4,1));

figure, 
subplot(3,1,1), 
plot(thist, xhist(:,2), thist, nominal_x_shown(:,2), 'r'); ylabel('Z(m)'), xlabel('time(s)');
title({'Nonlinear model',['u0: ',num2str(x0(3,1)),'(m/s) / Z0: ',num2str(x0(2,1)),'(m) / Desired Z: ',num2str(double(nominal_x(2,1))),'(m) / Desired u: ',num2str(double(nominal_x(3,1))),'(m/s)'], ...
      ['Kp: ', num2str(KpGains(1,:)), ' / ', num2str(KpGains(2,:))], ['Kd: ', num2str(KdGains(1,:)), ' / ', num2str(KdGains(2,:))]});
subplot(3,1,2),
plot(thist, xhist(:,3), thist, nominal_x_shown(:,3), 'r'); ylabel('u(m/s)');
subplot(3,1,3),
plot(thist, xhist(:,4), thist, nominal_x_shown(:,4), 'r'); ylabel('w(m/s)');

% figure, 
% subplot(3,1,1),
% plot(thist_LinearDet, xhist_LinearDet(:,2)); ylabel('Predicted Deviation Z(m)'), xlabel('time(s)');
% title({'Predicted deviation using the linear feedback method',['u0: ',num2str(x0(3,1)),'(m/s) / Z0: ',num2str(x0(2,1)),'(m)  / Desired Z: ',num2str(double(nominal_x(2,1))),'(m) / Desired u: ',num2str(double(nominal_x(3,1))),'(m/s)']});
% subplot(3,1,2),
% plot(thist_LinearDet, xhist_LinearDet(:,3)); ylabel('Predicted Deviation u(m/s)');
% subplot(3,1,3),
% plot(thist_LinearDet, xhist_LinearDet(:,4)); ylabel('Predicted Deviation w(m/s)');
% 
% figure, 
% subplot(3,1,1), 
% plot(thist_open, xhist_open(:,2), thist_open, nominal_x_shown(:,2), 'r'); ylabel('Z(m)'), xlabel('time(s)');
% title({'Nonlinear model using linear feedback method',['u0: ',num2str(x0(3,1)),'(m/s) / Z0: ',num2str(x0(2,1)),'(m)  / Desired Z: ',num2str(double(nominal_x(2,1))),'(m) / Desired u: ',num2str(double(nominal_x(3,1))),'(m/s)']});
% subplot(3,1,2),
% plot(thist_open, xhist_open(:,3), thist_open, nominal_x_shown(:,3), 'r'); ylabel('u(m/s)');
% subplot(3,1,3),
% plot(thist_open, xhist_open(:,4), thist_open, nominal_x_shown(:,4), 'r'); ylabel('w(m/s)');
% 
% figure, 
% subplot(3,1,1), 
% plot(thist_open, xhist_open(:,2)-nominal_x_shown(:,2)); ylabel('Real deviation Z(m)'), xlabel('time(s)');
% title({'Real deviation using the linear feedback method',['u0: ',num2str(x0(3,1)),'(m/s) / Z0: ',num2str(x0(2,1)),'(m)  / Desired Z: ',num2str(double(nominal_x(2,1))),'(m) / Desired u: ',num2str(double(nominal_x(3,1))),'(m/s)']});
% subplot(3,1,2),
% plot(thist_open, xhist_open(:,3)-nominal_x_shown(:,3)); ylabel('Real deviation u(m/s)');
% subplot(3,1,3),
% plot(thist_open, xhist_open(:,4)-nominal_x_shown(:,4)); ylabel('Real deviation w(m/s)');