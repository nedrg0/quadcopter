%% Animation ? 
x = squeeze(out.r.Data(1, 1, :));
y = squeeze(out.r.Data(2, 1, :));
z = squeeze(out.r.Data(3, 1, :));

phi = squeeze(out.angles.Data(1, 1, :));
theta = squeeze(out.angles.Data(2, 1, :));
psi = squeeze(out.angles.Data(3, 1, :));
trajectory = [x y z];          % [x y z], Nx3
angles     = [phi theta psi];      % [roll pitch yaw], radians, Nx3
N = length(x);


anglesInDegrees = false;
if anglesInDegrees
    angles = deg2rad(angles);
end
 
% ---- BOX GEOMETRY (defined in the body's local frame, centered at origin) ----
L = .5; W = .5; H = .25;     % length (x), width (y), height (z) -- set L=W=H for a cube
hx = L/2; hy = W/2; hz = H/2;
 
verts0 = [ -hx -hy -hz;    % 8 corners
            hx -hy -hz;
            hx  hy -hz;
           -hx  hy -hz;
           -hx -hy  hz;
            hx -hy  hz;
            hx  hy  hz;
           -hx  hy  hz ];
 
faces = [1 2 3 4;          % bottom
         5 6 7 8;          % top
         1 2 6 5;          % side
         2 3 7 6;          % side
         3 4 8 7;          % side
         4 1 5 8];         % side
 
% ---- FIGURE SETUP ----
figure('Color','w');
hold on; grid on; axis equal; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
 
pad = max([L W H]) + 0.5;
xlim([min(trajectory(:,1))-pad, max(trajectory(:,1))+pad]);
ylim([min(trajectory(:,2))-pad, max(trajectory(:,2))+pad]);
zlim([min(trajectory(:,3))-pad, max(trajectory(:,3))+pad]);
 
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'Color',[0.75 0.75 0.75]);
 
boxPatch = patch('Vertices', verts0, 'Faces', faces, ...
                  'FaceColor', [0.2 0.6 0.9], 'FaceAlpha', 0.7, 'EdgeColor','k');
 
trailLine = plot3(NaN,NaN,NaN,'b-','LineWidth',1.5);
trailX = []; trailY = []; trailZ = [];
 
% ---- ANIMATION LOOP ----
pause(0.5)
for k = 1:N
    pos   = trajectory(k,:);
    roll  = angles(k,1);
    pitch = angles(k,2);
    yaw   = angles(k,3);
 
    R = eulerToRotm(roll, pitch, yaw);   % local function, no toolbox needed
 
    vertsWorld = (R * verts0')' + pos;   % rotate then translate
    set(boxPatch, 'Vertices', vertsWorld);
 
    trailX(end+1) = pos(1); %#ok<SAGROW>
    trailY(end+1) = pos(2); %#ok<SAGROW>
    trailZ(end+1) = pos(3); %#ok<SAGROW>
    set(trailLine, 'XData', trailX, 'YData', trailY, 'ZData', trailZ);
 
    title(sprintf('Step %d / %d', k, N));
    drawnow;
    pause(0.02);   % adjust animation speed
end
 
% ---- LOCAL FUNCTION: Euler angles -> rotation matrix (ZYX / yaw-pitch-roll) ----
function R = eulerToRotm(roll, pitch, yaw)
    Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R = Rz * Ry * Rx;
end