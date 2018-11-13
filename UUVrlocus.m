clear;

syms t s
nominal_x0 = [0; 0.5; 5; 0]; % nominal initial state: X, Z, u, w
nominal_x = [nominal_x0(3,1)*t+nominal_x0(1,1); nominal_x0(2,1); nominal_x0(3,1); nominal_x0(4,1)]; % nominal solution
[A,B,nominal_input] = getLinearSys(nominal_x);

A = double(subs(A,t,0));
B = double(subs(B,t,0));
C = eye(size(A,2));
D = zeros(size(B,1),size(B,2));

[bx,ax] = ss2tf(A,B,C,D,1);
Xh = tf(bx(1,:),ax);
figure, rlocus(Xh);
Zh = tf(bx(2,:),ax);
figure, rlocus(Zh);
uh = tf(bx(3,:),ax);
figure, rlocus(uh);
wh = tf(bx(4,:),ax);
figure, rlocus(wh);

[bz,az] = ss2tf(A,B,C,D,2);
Xh = tf(bz(1,:),az);
figure, rlocus(Xh);
Zh = tf(bz(2,:),az);
figure, rlocus(Zh);
uh = tf(bz(3,:),az);
figure, rlocus(uh);
wh = tf(bz(4,:),az);
figure, rlocus(wh);