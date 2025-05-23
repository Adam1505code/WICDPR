%% wicdpr_2d_pid_roll_ZN.m
% 2-Cable WICDPR in 2D with Z-N PID for translation + PD for roll orientation.
%
% Platform: ~30cm wide x 10cm tall, but corners attach at top-left / top-right.
% Anchors: (0, H) and (Lx, H).
%
% State: [ x, y, phi, vx, vy, omega ]
%   x,y   => platform COM in 2D
%   phi   => roll angle (radians)
%   vx,vy => velocities
%   omega => roll rate
%
% By: ADAM MCENEANEY, 20/11/2024

clear; clc; close all;

%% 1) Workspace and user inputs
Lx = input('Enter horizontal workspace limit Lx (m): ');
H  = input('Enter top anchor height H (m): ');
if Lx <= 0 || H <= 0
    error('Lx and H must be positive.');
end

% Enter target within [0..Lx, 0..H], typically below anchors
xt = input(sprintf('Enter target x (0..%.2f): ',Lx));
yt = input(sprintf('Enter target y (0..%.2f): ',H));
if (xt<0 || xt>Lx || yt<0 || yt>H)
    error('Target outside the workspace boundaries.');
end
target = [xt; yt];

%% 2) Physical & Simulation Params
w_platform = 0.30;  % 30 cm wide
h_platform = 0.10;  % 10 cm tall (not heavily used except for drawing)
m = 2.7;            % total mass, [kg]
g = 9.81;           % gravity [m/s^2]
b_lin = 2.0;        % linear damping
dt = 0.01;          % time step
maxSteps = 10000;   % max iteration
posTol   = 0.003;   % settle threshold (3 mm)

% Inertia about the COM for "roll" (approx. rod or box shape)
% For a 30x10 cm rectangle, axis out-of-plane => I ~ (1/12)*m*(width^2)
Izz = (1/12)*m*(w_platform^2 + h_platform^2);
b_rot = 0.5;   % rotational damping

%% 3) Ziegler-Nichols Gains for (x,y)
Ku = input('Enter ultimate gain Ku for ZN: ');
Pu = input('Enter ultimate period Pu for ZN: ');
if Ku<=0 || Pu<=0
    error('Invalid Ku or Pu.');
end

% Classic PID from Z-N:
Kp_zn = 0.6*Ku;
Ti_zn = 0.5*Pu;     % => Ki = Kp / Ti
Ki_zn = Kp_zn / Ti_zn;
Kd_zn = 0.125*Pu*Kp_zn;

% We'll use diagonal gains for x,y
Kp_pid = diag([Kp_zn,Kp_zn]);
Ki_pid = diag([Ki_zn,Ki_zn]);
Kd_pid = diag([Kd_zn,Kd_zn]);

fprintf('\nUsing ZN Gains for translation:\n');
fprintf('Kp=%.3f, Ki=%.3f, Kd=%.3f\n',Kp_zn, Ki_zn, Kd_zn);

%% 4) Orientation PD Gains (roll)
Kp_phi = 5.0;
Kd_phi = 2.0;

%% 5) Anchor points
anchor1 = [0;   H];
anchor2 = [Lx; H];

%% 6) Corner offsets from the COM for top-left and top-right
% We'll only track top edge corners for cable attachments
cornerLocal_L = [-0.5*w_platform; +0.0];
cornerLocal_R = [+0.5*w_platform; +0.0];

%% 7) Initialize state
% state = [x, y, phi, vx, vy, omega]
x0 = Lx/2;   % start mid
y0 = 0.2*H;  % some fraction of the height
phi0 = 0;
vx0 = 0; vy0=0; om0=0;
state = [x0; y0; phi0; vx0;vy0;om0];

int_err = zeros(2,1);
prev_err= zeros(2,1);

%% For storing logs
Nmax = maxSteps;
time_log    = NaN(1,Nmax);
pos_log     = NaN(2,Nmax);
err_log     = NaN(1,Nmax);
roll_log    = NaN(1,Nmax);
cableLen_log= NaN(2,Nmax);
tension_log = NaN(2,Nmax);

settled = false;
settleTime = NaN;

%% 8) Setup Figures
fh = figure('Name','2D WICDPR with ZN + PD Roll');
ax = axes(fh); hold(ax,'on'); grid(ax,'on');
axis(ax,[0 Lx 0 H]); axis(ax,'equal');
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');
title(ax,'2D WICDPR (Ziegler-Nichols for translation + PD for roll)');

% Plot boundary
plot([0 Lx],[0 0],'k--','LineWidth',1);
plot([0 0],[0 H],'k--','LineWidth',1);
plot([Lx Lx],[0 H],'k--','LineWidth',1);
plot([0 Lx],[H H],'k--','LineWidth',1);

hCornerL = plot(ax,0,0,'ro-','LineWidth',2);
hCornerR = plot(ax,0,0,'ro-','LineWidth',2);
hPlat = plot(ax,0,0,'b-','LineWidth',3);

plot(ax,xt,yt,'kx','LineWidth',2,'MarkerSize',8);

%% 9) Main loop
for step=1:maxSteps
    t_now = (step-1)*dt;
    time_log(step) = t_now;

    % Extract states
    x   = state(1);
    y   = state(2);
    phi = state(3);
    vx  = state(4);
    vy  = state(5);
    om  = state(6);

    pos       = [x;y];
    e         = target - pos;       % (x,y) error
    err_log(step) = norm(e);

    % PD in x,y with ZN-based PID (Kp,Ki,Kd) [we skip integral if you prefer]
    de = (e - prev_err)/dt;
    int_err = int_err + e*dt;  % integral
    prev_err= e;

    F_pid = Kp_pid*e + Ki_pid*int_err + Kd_pid*de;
    % net force minus linear damping + gravity in negative y
    % but here let's do purely F=(F_pid - b_lin*[vx;vy]) + (0,-mg).
    F = F_pid - b_lin*[vx; vy] + [0; m*g];

    % acceleration
    ax_ = (F(1))/m;
    ay_ = (F(2) - m*g)/m;  % subtract mg

    % corners in local -> global
    R = [ cos(phi), -sin(phi);
          sin(phi),  cos(phi)];
    cL = [x;y] + R*cornerLocal_L;
    cR = [x;y] + R*cornerLocal_R;

    % cable directions
    vL = anchor1 - cL;
    vR = anchor2 - cR;
    lenL = norm(vL);
    lenR = norm(vR);
    if lenL>1e-9, uL=vL/lenL; else uL=[0;0]; end
    if lenR>1e-9, uR=vR/lenR; else uR=[0;0]; end

    % T1,T2 that produce net force = F (2x1) if possible:
    A2x2 = [uL(1), uR(1);
            uL(2), uR(2)];
    if rank(A2x2)<2
        Tvals = [0;0];
    else
        Tvals = A2x2 \ F;
    end
    T1 = Tvals(1);  T2 = Tvals(2);

    % sum torque about COM
    % corner offsets in global
    rL = (cL - [x;y]);
    rR = (cR - [x;y]);
    tauRoll = cross2D(rL, T1*uL) + cross2D(rR, T2*uR);
    % orientation PD => keep phi near 0
    tauCtrl = -Kp_phi*phi - Kd_phi*om;
    tauNet  = tauRoll + tauCtrl - (b_rot*om);

    % rotational accel
    om_dot = tauNet / Izz;

    % Integrate
    vx = vx + ax_*dt;
    vy = vy + ay_*dt;
    x  = x  + vx*dt;
    y  = y  + vy*dt;

    om = om + om_dot*dt;
    phi= phi+ om*dt;

    % store back
    state = [x;y;phi;vx;vy;om];

    % clamp to workspace?
    x = max(0, min(Lx, x));
    y = max(0, min(H, y));
    state(1)=x; state(2)=y;

    % logs
    pos_log(:,step)     = [x;y];
    roll_log(step)      = phi;
    cableLen_log(:,step)= [lenL; lenR];
    tension_log(:,step) = [T1; T2];

    % check settle
    speedMag = hypot(vx, vy);
    if ~settled && (err_log(step)<=posTol) && (speedMag<=0.01)
        settled=true;
        settleTime=t_now;
    end

    % update plot
    if mod(step,20)==1
        % corners again
        cL = [x;y] + R*cornerLocal_L;
        cR = [x;y] + R*cornerLocal_R;
        set(hCornerL,'XData',[anchor1(1), cL(1)], 'YData',[anchor1(2), cL(2)]);
        set(hCornerR,'XData',[anchor2(1), cR(1)], 'YData',[anchor2(2), cR(2)]);
        % draw platform top edge
        set(hPlat,'XData',[cL(1), cR(1)], 'YData',[cL(2), cR(2)]);
        drawnow limitrate
    end
end

if settled
    fprintf('\nSettled at t=%.3f s\n',settleTime);
else
    fprintf('\nNot settled after %d steps (%.1f s)\n',maxSteps, maxSteps*dt);
end

%% 10) post-plots
Tend = min(step,maxSteps);
tt  = (0:Tend-1)*dt;

figure('Name','2D WICDPR Results'); 
subplot(3,1,1); hold on; grid on;
plot(tt, pos_log(1,1:Tend),'r','LineWidth',1.5);
plot(tt, pos_log(2,1:Tend),'b','LineWidth',1.5);
legend('X','Y','Location','best');
ylabel('Position (m)'); xlabel('Time (s)');
title('Platform Position vs Time');

subplot(3,1,2); hold on; grid on;
plot(tt, roll_log(1:Tend)*180/pi,'m','LineWidth',1.5);
ylabel('Roll (deg)'); xlabel('Time (s)');
title('Roll Orientation');

subplot(3,1,3); hold on; grid on;
plot(tt, err_log(1:Tend),'k','LineWidth',1.3);
ylabel('Error (m)'); xlabel('Time (s)');
title('Position Error');

figure('Name','Cable Info'); 
subplot(2,1,1); hold on; grid on;
plot(tt, cableLen_log(1,1:Tend),'r','LineWidth',1.2);
plot(tt, cableLen_log(2,1:Tend),'b','LineWidth',1.2);
legend('Cable1','Cable2','Location','best');
ylabel('Cable Length (m)'); xlabel('Time (s)');
title('Cable Lengths');

subplot(2,1,2); hold on; grid on;
plot(tt, tension_log(1,1:Tend),'r','LineWidth',1.2);
plot(tt, tension_log(2,1:Tend),'b','LineWidth',1.2);
legend('T1','T2','Location','best');
ylabel('Cable Tension (N)'); xlabel('Time (s)');
title('Cable Tensions');

%% Helper: cross2D
% cross2D([x1;y1],[x2;y2]) => "z-component" of 3D cross
function zc = cross2D(a,b)
% a,b in R^2 => cross is out-of-plane
zc = a(1)*b(2) - a(2)*b(1);
end
