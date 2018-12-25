clear;
N_states = 4;
N_inputs = 2;
tspan = (0:0.5:300)'; % ode time span
syms t

%%%%%%% nominal data setting (usually equilibrium) to get the linearized system
%%%%%%% and the nominal solution
% Note that X doesn't influence the states
% We have 3 system-dependent states, and 5 variables to set to get the
% desired motion. Therefore we need to set 2 parameters to get the rest of
% the steady motion states and inputs
nominal_Z_setting = 0.5;
nominal_u_setting = 2;
nominal_x0 = [0; nominal_Z_setting; nominal_u_setting; 0]; % nominal initial state: X, Z, u, w
nominal_data_setting = [nominal_Z_setting; nominal_u_setting]; % Set Z, u. 
[A,B,nominal_x,nominal_input] = getLinearSys(nominal_x0,nominal_data_setting);

%%%%%%% Gains and initial states
DevPercentage = 1;
%x0 = nominal_x0*(1+0.01*DevPercentage); % initial state: X, Z, u, w
x0 = [0; 0.6; 2; 0]; % initial state: X, Z, u, w
% Control gains
% feedback error
K_udir = [0 0 12.1896 0];
K_wdir = [0 162.3276 0 200.2832];
KGains = [K_udir; K_wdir];

%%%%%% solve ode of nonlinear system
UUVode = @(tt,x) systemUUV(tt,x,nominal_x,nominal_input,KGains);
[thist,xhist] = ode45(UUVode,tspan,x0);
%%%%%% solve ode of linearized system
UUVode_Linear = @(tt,Detx) systemUUVLinear(tt,Detx,A,B,KGains);
[thist_LinearDet,xhist_LinearDet] = ode45(UUVode_Linear,tspan,x0-nominal_x0);


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
plot(thist, xhist(:,2), '-o', thist, nominal_x_shown(:,2), 'r'); hold on;
plot(thist_LinearDet, xhist_LinearDet(:,2)+nominal_x(2,1), 'g', 'LineWidth', 2); ylabel('Z(m)'), xlabel('time(s)'),
title({'Model simulation',['Devx0: ',num2str(DevPercentage),'(percent) / Desired Z: ',num2str(double(nominal_x(2,1))),'(m) / Desired u: ',num2str(double(nominal_x(3,1))),'(m/s)'], ...
      ['K: ', num2str(KGains(1,:)), ' / ', num2str(KGains(2,:))]});
legend('nonlinear', 'nominal trajectory', 'linear')
subplot(3,1,2),
plot(thist, xhist(:,3), '-o', thist, nominal_x_shown(:,3), 'r'); hold on;
plot(thist_LinearDet, xhist_LinearDet(:,3)+nominal_x(3,1), 'g', 'LineWidth', 2); ylabel('u(m/s)'), xlabel('time(s)');
subplot(3,1,3),
plot(thist, xhist(:,4), '-o', thist, nominal_x_shown(:,4), 'r'); hold on;
plot(thist_LinearDet, xhist_LinearDet(:,4)+nominal_x(4,1), 'g', 'LineWidth', 2); ylabel('w(m/s)'), xlabel('time(s)');


% figure, 
% subplot(3,1,1),
% plot(thist_LinearDet, xhist_LinearDet(:,2)); ylabel('Predicted Deviation Z(m)'), xlabel('time(s)');
% title({'Predicted deviation using the linear feedback method',['u0: ',num2str(x0(3,1)),'(m/s) / Z0: ',num2str(x0(2,1)),'(m)  / Desired Z: ',num2str(double(nominal_x(2,1))),'(m) / Desired u: ',num2str(double(nominal_x(3,1))),'(m/s)']});
% subplot(3,1,2),
% plot(thist_LinearDet, xhist_LinearDet(:,3)); ylabel('Predicted Deviation u(m/s)');
% subplot(3,1,3),
% plot(thist_LinearDet, xhist_LinearDet(:,4)); ylabel('Predicted Deviation w(m/s)');

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