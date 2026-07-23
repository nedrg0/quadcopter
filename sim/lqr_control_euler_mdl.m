%% Servomechanism design
clear; clc;
quadcopter_params;

psi0 = 0;   % nominal hover heading
r0 = [1; 1; 1];
v0 = [0; 0; 0];
Theta0 = [0; 0; psi0];
w0 = [0; 0; 0];
F0 = m * g/ 4;

x0 = [r0; v0; Theta0; w0];

r1 = [2; 2; 3];
x1 = [r1; v0; Theta0; w0];

u0 = [F0; F0; F0; F0];
% Reduced 12-state model: [x y z xdot ydot zdot phi theta psi p q r]
c = cos(psi0); s = sin(psi0);

A = zeros(12,12);
A(1,4)=1; A(2,5)=1; A(3,6)=1;                  % r' = v
A(4,8)=g*c; A(4,7)=g*s;                        % xddot = g*c*theta + g*s*phi
A(5,8)=g*s; A(5,7)=-g*c;                       % yddot = g*s*theta - g*c*phi
A(7,10)=1; A(8,11)=1; A(9,12)=1;               % Theta' = omega

B = zeros(12,4);
B(6,:) = 1/m;                                  % zddot = du1/m
B(10,2)=1/Ixx; B(10,4)=-1/Ixx;                 % phiddot
B(11,1)=1/Iyy; B(11,3)=-1/Iyy;                 % thetaddot
B(12,1)=gamma/Izz; B(12,2)=-gamma/Izz; B(12,3)=gamma/Izz; B(12,4)=-gamma/Izz;  % psiddot

fprintf('rank(A,B) = %d / 12\n', rank(ctrb(A,B)));

% C selects ONLY x, y, z, psi
C = zeros(4,12);
C(1,1)=1; C(2,2)=1; C(3,3)=1; C(4,9)=1;   % x, y, z, psi

% Servo augmentation
r = size(C,1); n = size(A,1); mI = size(B,2);
Ae = [zeros(r,r), C;
      zeros(n,r), A];
Be = [zeros(r ,mI); B];

fprintf('rank(Ae,Be) = %d / %d\n', rank(ctrb(Ae,Be)), size(Ae,1));

% LQR gains
Q = eye(size(Ae,1)) ;
R = eye(size(Be,2)) ;
K = lqr(Ae, Be, Q, R);

eigCL = eig(Ae - Be*K);
fprintf('max real(eig(closed loop)) = %.4f  (must be < 0)\n', max(real(eigCL)));