%% Quadcopter params.
% Gravitational force
g= 9.81;
G = [0 ; 0; -g];

% Interia matrix
Ixx = 2.32 * 1e-3;
Iyy = 2.32 * 1e-3;
Izz = 4 * 1e-3;
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

% Motor arm length
L = 0.175;

% Mass
m = 0.5;

% Motor constants
kF = 6.11 * 1e-8;
kM = 1.5 * 1e-9;
gamma = kM/kF;
wi_max = 16000;

% Input gain matrix
M = [1 1 1 1; 0 L 0 -L; L 0 -L 0; gamma -gamma gamma -gamma];




