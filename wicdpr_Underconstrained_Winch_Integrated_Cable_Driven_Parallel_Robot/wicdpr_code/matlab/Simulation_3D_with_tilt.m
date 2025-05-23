% wicdpr_3d_pid_tilt_ZN.m
% 3D WICDPR with PID (Ziegler–Nichols) for translation + PD for pitch/roll.
%
% Features:
%   - 30x30x10 cm platform
%   - Plant pots (visual distractors)
%   - Cable tensions displayed
%   - Basic orientation model (roll, pitch) with torque from cables
%   - Z–N gains for translation, plus orientation PD to keep platform level
%
% By Adam McEneaney, [10/10/2024]

clear; clc; close all;

%% 1) User inputs for workspace
Lx = input('Enter workspace X limit (m): ');
Ly = input('Enter workspace Y limit (m): ');
Lz = input('Enter workspace Z limit (m): ');
if any([Lx Ly Lz] <= 0), error('Workspace limits must be positive.'); end

target = input('Enter target [x y z] within [0 Lx]×[0 Ly]×[0 Lz]: ');
if any(target < 0) || target(1) > Lx || target(2) > Ly || target(3) > Lz
    error('Target outside workspace.');
end
target = target(:);

%% 2) Physical & simulation parameters
% Platform geometry
w = 0.30;  % [m] width  (x-dim)
d = 0.30;  % [m] depth  (y-dim)
h = 0.10;  % [m] height (z-dim)

m = 4.5;     % mass [kg]
b = 2.0;     % linear damping [N·s/m]
g = 9.81;    % gravity [m/s^2]

dt       = 0.01;   % timestep [s]
maxSteps = 10000;  % max iteration steps
posTol   = 0.005;  % settle tolerance (5 mm)

% Anchor positions in the corners of the top bounding rectangle
anchors = [0,0,Lz;  Lx,0,Lz;  0,Ly,Lz;  Lx,Ly,Lz];

% Local corner offsets from platform COM
offsets = [...
  -w/2, -d/2, -h/2;  % corner 1 (left-back)
   w/2, -d/2, -h/2;  % corner 2 (right-back)
  -w/2,  d/2, -h/2;  % corner 3 (left-front)
   w/2,  d/2, -h/2]; % corner 4 (right-front)

% Rotational inertia (assuming uniform box)
Ix = (1/12)*m*(d^2 + h^2);  % about X-axis (roll)
Iy = (1/12)*m*(w^2 + h^2);  % about Y-axis (pitch)
b_rot = 0.5;                % rotational damping (tunable)

print_interval = round(0.1/dt);

%% 3) Ziegler–Nichols Gains for Translation
% User-provided Ku (ultimate gain) and Pu (ultimate oscillation period).
Ku = input('Enter ultimate gain Ku for ZN: ');
Pu = input('Enter ultimate period Pu for ZN: ');
if Ku<=0 || Pu<=0
    error('Ku and Pu must be positive for ZN.');
end

% ZN: Classic PID formula
Kp_zn = 0.6 * Ku;
Ti_zn = 0.5 * Pu;            % integral time
Ki_zn = Kp_zn / Ti_zn;       % or 2*Kp_zn / Pu
Kd_zn = 0.125 * Pu * Kp_zn;

% Build diagonal gain matrices for x,y,z:
Kp_pid = diag([Kp_zn, Kp_zn, Kp_zn]);
Ki_pid = diag([Ki_zn, Ki_zn, Ki_zn]);
Kd_pid = diag([Kd_zn, Kd_zn, Kd_zn]);

fprintf('\n*** Using ZN Gains for translation ***\n');
fprintf('Ku=%.2f, Pu=%.2f => Kp=%.2f, Ki=%.2f, Kd=%.2f\n',...
    Ku, Pu, Kp_zn, Ki_zn, Kd_zn);

%% 4) Orientation PD Gains (roll, pitch)
% We want to keep phi=0, theta=0. Gains can be tuned.
Kp_phi   = 5;  % PD for roll
Kd_phi   = 2;
Kp_theta = 5;  % PD for pitch
Kd_theta = 2;

%% 5) Precompute desired cable lengths at target (no tilt assumption)
desired_verts_tgt = bsxfun(@plus, offsets, target');
desired_Ls_tgt    = sqrt(sum((anchors - desired_verts_tgt).^2,2));

%% 6) Initialize state
% state = [ x, y, z, phi, theta, vx, vy, vz, p, q ]
% Start near center, with no orientation or velocity
state = [ Lx/2;  Ly/2;  h/2;  0;  0;  0; 0; 0;  0; 0 ];

int_err  = zeros(3,1);  % integral error for x,y,z
prev_err = zeros(3,1);

%% 7) Data storage
com_hist       = nan(3,maxSteps);
err_hist       = nan(1,maxSteps);
len_hist       = nan(4,maxSteps);
spd_hist       = nan(4,maxSteps);
cable_err_hist = nan(4,maxSteps);
tension_hist   = nan(4,maxSteps);
pitch_hist     = nan(1,maxSteps);
roll_hist      = nan(1,maxSteps);

settled     = false;
settle_time = NaN;
overshoot   = 0;
dist0       = norm(target - state(1:3));

%% 8) Visualization setup
fig = figure('Renderer','opengl'); 
ax  = axes(fig); 
hold(ax,'on');  grid(ax,'on');  view(ax,3);
axis(ax,[0 Lx 0 Ly 0 Lz]); 
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D WICDPR (PID with ZN + PD Tilt Control)');

% bounding box lines
plot3(ax,[0 Lx Lx 0 0 0 Lx Lx],...
      [0 0 Ly Ly 0 0 0 0],...
      [0 0 0 0 Lz Lz Lz Lz],'k--');

% Add "plant pots" for fun
potPositions = [0.5*Lx, 0.5*Ly;
                0.2*Lx, 0.8*Ly;
                0.8*Lx, 0.2*Ly];
potRadius = 0.05;
potHeight = 0.10;
nPots = size(potPositions,1);

for i=1:nPots
    [xx,yy,zz] = cylinder(potRadius,16);
    zz = zz*potHeight;
    xx = xx + potPositions(i,1);
    yy = yy + potPositions(i,2);
    surf(ax,xx,yy,zz,'FaceColor',[0.6 0.3 0.1],'EdgeColor','none');
end

% Initial corners & cables
init_verts = computeCorners(state, offsets);
hPlat = patch(ax,'Vertices',init_verts,'Faces',[1 2 4 3],...
    'FaceColor','r','FaceAlpha',0.3);
hCable = cell(4,1);
for i=1:4
    hCable{i} = plot3(ax,[anchors(i,1), init_verts(i,1)],...
                         [anchors(i,2), init_verts(i,2)],...
                         [anchors(i,3), init_verts(i,3)],...
                         'Color','r','LineWidth',1.5);
end
drawnow;

%% 9) Main simulation loop
step = 0;

while step < maxSteps && ~settled
    step = step + 1;
    t_now = (step-1)*dt;

    %-----------------------
    %  Extract states
    %-----------------------
    xPos   = state(1);
    yPos   = state(2);
    zPos   = state(3);
    phi    = state(4);  % roll
    theta  = state(5);  % pitch
    vx     = state(6);
    vy     = state(7);
    vz     = state(8);
    p      = state(9);  % roll rate
    q      = state(10); % pitch rate

    pos = [xPos; yPos; zPos];
    vel = [vx; vy; vz];

    %-----------------------
    %  Translational PID
    %-----------------------
    e = target - pos;            % position error
    int_err = int_err + e*dt;
    de = (e - prev_err)/dt;
    prev_err = e;

    Fpid = Kp_pid*e + Ki_pid*int_err + Kd_pid*de;
    % Net force (minus damping and plus gravity, etc.)
    F = Fpid - b*vel + [0; 0; m*g];

    % acceleration
    a = (F - [0;0;m*g] - b*vel) / m;

    %-----------------------
    %  Compute corners
    %-----------------------
    verts = computeCorners(state, offsets);

    %-----------------------
    %  Cable directions + tension
    %-----------------------
    u = zeros(3,4);
    for i=1:4
        vec = anchors(i,:)' - verts(i,:)';
        nrm = norm(vec);
        if nrm > 1e-9
            u(:,i) = vec / nrm;
        end
    end
    A = u;  % 3×4

    if rank(A) < 3
        Tvals = zeros(4,1);
    else
        % Solve for tensions that produce net force ~ F
        % Very rough approach: T = A'*(A*A')^-1 * F
        Tvals = A'*((A*A')\F);
    end

    %-----------------------
    %  Torque from cables
    %-----------------------
    torque = [0;0;0];
    for i=1:4
        cornerVec = verts(i,:)' - pos;  % from COM to corner i
        forceVec  = Tvals(i)*u(:,i);
        torque    = torque + cross(cornerVec, forceVec);
    end
    tau_x = torque(1); % roll torque
    tau_y = torque(2); % pitch torque

    %-----------------------
    %  Orientation PD
    %  (Keeps phi,theta near 0)
    %-----------------------
    tau_x_ctrl = -Kp_phi*phi - Kd_phi*p;
    tau_y_ctrl = -Kp_theta*theta - Kd_theta*q;

    tau_x = tau_x + tau_x_ctrl;
    tau_y = tau_y + tau_y_ctrl;

    %-----------------------
    %  Rotational dynamics
    %-----------------------
    p_dot = (1/Ix)*(tau_x - b_rot*p);
    q_dot = (1/Iy)*(tau_y - b_rot*q);

    %-----------------------
    %  Integrate everything
    %-----------------------
    vx = vx + a(1)*dt;
    vy = vy + a(2)*dt;
    vz = vz + a(3)*dt;

    xPos = xPos + vx*dt;
    yPos = yPos + vy*dt;
    zPos = zPos + vz*dt;

    p = p + p_dot*dt;
    q = q + q_dot*dt;

    phi   = phi   + p*dt;
    theta = theta + q*dt;

    % clamp translation to workspace
    xPos = max(0, min(Lx, xPos));
    yPos = max(0, min(Ly, yPos));
    zPos = max(0, min(Lz, zPos));

    % update state
    state = [ xPos; yPos; zPos; phi; theta; vx; vy; vz; p; q ];

    %-----------------------
    %  Logging
    %-----------------------
    com_hist(:,step) = pos;
    err_hist(step)   = norm(e);
    len_hist(:,step) = sqrt(sum((anchors - verts).^2,2));
    if step>1
        spd_hist(:,step) = (len_hist(:,step) - len_hist(:,step-1))/dt;
    end
    cable_err_hist(:,step) = len_hist(:,step) - desired_Ls_tgt;

    pitch_hist(step) = theta; 
    roll_hist(step)  = phi; 
    tension_hist(:,step) = Tvals;

    curDist = norm(e);
    if dist0>0 && curDist>dist0
        overshoot = max(overshoot, curDist - dist0);
    end

    %-----------------------
    %  Print occasionally
    %-----------------------
    if mod(step, print_interval)==1
        fprintf('\nTime = %.2f s\n', t_now);
        fprintf('  pos=(%.3f,%.3f,%.3f), err=%.3f m\n',...
                xPos,yPos,zPos, norm(e));
        fprintf('  roll=%.3f deg, pitch=%.3f deg\n',...
                roll_hist(step)*180/pi, pitch_hist(step)*180/pi);
        fprintf('  Tension=[%.2f %.2f %.2f %.2f] N\n', Tvals);
    end

    %-----------------------
    %  Check settling
    %-----------------------
    vSpeed = norm([vx; vy; vz]);
    if ~settled && (norm(e)<=posTol) && (vSpeed<=0.01)
        settled = true;
        settle_time = t_now;
    end

    %-----------------------
    %  Update animation
    %-----------------------
    set(hPlat,'Vertices',verts);
    for i=1:4
        set(hCable{i},...
            'XData',[anchors(i,1), verts(i,1)],...
            'YData',[anchors(i,2), verts(i,2)],...
            'ZData',[anchors(i,3), verts(i,3)]);
    end
    drawnow limitrate;
end

%% Final results
idx  = step;
time = (0:idx-1)*dt;

if settled
    fprintf('\nPlatform settled at t=%.2f s\n', settle_time);
else
    fprintf('\nReached maxSteps=%d without settling.\n',maxSteps);
end
fprintf('Overshoot (approx)=%.3f m\n', overshoot);

%% 10) Post-plots
% COM trajectory
figure; hold on; grid on; view(3);
plot3(com_hist(1,1:idx), com_hist(2,1:idx), com_hist(3,1:idx), 'r','LineWidth',1.5);
plot3(target(1),target(2),target(3),'kx','LineWidth',2);
xlabel('X'); ylabel('Y'); zlabel('Z');
legend({'PID (ZN + tilt ctrl)','Target'},'Location','best');
title('3D COM Trajectory (with Orientation PD)');

% Position error
figure; hold on; grid on;
plot(time, err_hist(1:idx),'r','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Error (m)');
title('COM Position Error (Ziegler–Nichols Gains)');

% Cable lengths
figure; hold on; grid on;
plot(time, len_hist(:,1:idx)','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Cable Length (m)');
legend('Cable1','Cable2','Cable3','Cable4','Location','best');
title('Cable Lengths');

% Cable speeds
figure; hold on; grid on;
plot(time, spd_hist(:,1:idx)','--','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Cable Speed (m/s)');
legend('Cable1','Cable2','Cable3','Cable4','Location','best');
title('Cable Speeds');

% Cable tensions
figure; hold on; grid on;
plot(time, tension_hist(:,1:idx)','LineWidth',1.2);
xlabel('Time (s)'); ylabel('Tension (N)');
legend('Cable1','Cable2','Cable3','Cable4','Location','best');
title('Cable Tensions');

% Pitch & Roll
figure; 
subplot(2,1,1); hold on; grid on;
plot(time, pitch_hist(1:idx)*180/pi,'r','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Pitch (deg)');
title('Platform Pitch');

subplot(2,1,2); hold on; grid on;
plot(time, roll_hist(1:idx)*180/pi,'b','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Roll (deg)');
title('Platform Roll');

%% Local function: computeCorners
function verts = computeCorners(state, offsets)
    % state = [x, y, z, phi, theta, vx, vy, vz, p, q]
    xPos  = state(1);
    yPos  = state(2);
    zPos  = state(3);
    phi   = state(4); % roll
    theta = state(5); % pitch

    R_roll = [1, 0,         0
              0, cos(phi), -sin(phi)
              0, sin(phi),  cos(phi)];

    R_pitch= [ cos(theta), 0, sin(theta)
               0,          1, 0
              -sin(theta), 0, cos(theta)];

    R = R_pitch * R_roll;  % or adjust order if needed

    verts = zeros(size(offsets));
    for k=1:size(offsets,1)
        cornerLocal  = offsets(k,:)';
        cornerGlobal = [xPos; yPos; zPos] + R*cornerLocal;
        verts(k,:)   = cornerGlobal';
    end
end
