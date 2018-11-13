x0 = [0; 0.5; 5; 0]; 
ref = [0; 0.5; 5; 0]; % desired values: X, Z, u, w
% Feedback gains: Kxdir, Kzdir
Kxdir = [0 0 1 0];
Kzdir = [0 1 0 2];
feedbackGain = [Kxdir; Kzdir];

tspan = (0:0.5:300)';
%%% ode45 for nonlinear system
UUVode45 = @(tt,x) systemUUV(tt,x,ref,feedbackGain);
[thist,xhist] = ode45(UUVode45,tspan,x0);

figure, 
subplot(3,1,1), title('Nonlinear model');
plot(thist, xhist(:,2)); ylabel('Z(m)'), xlabel('time(s)');
title(['u0: ',num2str(x0(3,1)),'(m/s) / Desired Z: ',num2str(ref(2,1)),'(m) / Desired u: ',num2str(ref(3,1)),'(m/s)']);
subplot(3,1,2),
plot(thist, xhist(:,3)); ylabel('u(m/s)');
subplot(3,1,3),
plot(thist, xhist(:,4)); ylabel('w(m/s)');