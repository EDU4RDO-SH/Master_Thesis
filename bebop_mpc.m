% In order to run this script you need matlab_rosbag package
% https://github.com/bcharrow/matlab_rosbag (source)
% https://github.com/bcharrow/matlab_rosbag/releases (binary)

% ======================== I M P O R T A N T! =============================
% Change BAG NAMES and VALID DATA RANGES for Experiment1 and Experiment2 
% if you want to test this program with other bag files.
% =========================================================================

clear, clc;
rad2deg = 180/pi;


%% Loading bags

path(path, '../read_bags');
path(path, '../helper_functions');

% bagfile
%bagfile = '../bags/bebop_mpc/bebop-mpc_2021-01-03-16-42-38.bag';    % 3D ladder
bagfile = '../bags/bebop_mpc/bebop-mpc_2021-01-03-16-53-55.bag';    % square


reference_topic = '/bebop/command/current_reference';
odometry_topic = '/bebop/odometry';
errors_topic = '/bebop/state_errors';
rpyt_topic = '/bebop/command/roll_pitch_yawrate_thrust';

bag = ros.Bag(bagfile);

%bag.info       % First experiment info:

%% Prepare datasets

% reference
Exp.reference = readCommandReference(bag, reference_topic);
Exp.reference.t = Exp.reference.t - Exp.reference.t(1);      % time from 0
Exp.reference.q = [Exp.reference.q(4,:); Exp.reference.q(1,:); Exp.reference.q(2,:); Exp.reference.q(3,:)]; % rearrange quaternion   
Exp.reference.rpy = quat2rpy(Exp.reference.q);  % convert rotation quaternion to Euler angles

% current states
Exp.odometry = readOdometry(bag, odometry_topic);
Exp.odometry.t = Exp.odometry.t - Exp.odometry.t(1);      % time from 0
Exp.odometry.q = [Exp.odometry.q(4,:); Exp.odometry.q(1,:); Exp.odometry.q(2,:); Exp.odometry.q(3,:)]; % rearrange quaternion   
Exp.odometry.rpy = quat2rpy(Exp.odometry.q);  % convert rotation quaternion to Euler angles

% errors
Exp.errors = readStateErrors(bag, errors_topic);
Exp.errors.t = Exp.errors.t - Exp.errors.t(1);      % time from 0

% control inputs
Exp.rpyt = readCommandRollPitchYawRateThrust(bag, rpyt_topic);
Exp.rpyt.t = Exp.rpyt.t - Exp.rpyt.t(1);      % time from 0


%% Plotting

% position ****************************************************************
figure(1)
subplot(3,1,1)
plot(Exp.reference.t, Exp.reference.p(1,:), 'b', Exp.odometry.t, Exp.odometry.p(1,:), 'r--', 'linewidth', 2); grid on;
ylabel('x [m]');
legend('x_r', 'x');
subplot(3,1,2)
plot(Exp.reference.t, Exp.reference.p(2,:), 'b', Exp.odometry.t, Exp.odometry.p(2,:), 'r--', 'linewidth', 2); grid on;
ylabel('y [m]')
legend('y_r', 'y');
subplot(3,1,3)
plot(Exp.reference.t, Exp.reference.p(3,:), 'b', Exp.odometry.t, Exp.odometry.p(3,:), 'r--', 'linewidth', 2); grid on;
ylabel('z [m]')
legend('z_r', 'z');
xlabel('t [sec]');

figure(2)
plot3(Exp.reference.p(1,:), Exp.reference.p(2,:), Exp.reference.p(3,:), 'b', 'linewidth', 2); grid on;
hold on
plot3(Exp.odometry.p(1,:), Exp.odometry.p(2,:), Exp.odometry.p(3,:), 'r--', 'linewidth', 2); grid on;
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')



% position errors
figure(3)
plot(Exp.errors.t, Exp.errors.p(1,:), 'r', 'linewidth', 2); grid on;
hold on
plot(Exp.errors.t, Exp.errors.p(2,:), 'g', 'linewidth', 2); grid on;
hold on
plot(Exp.errors.t, Exp.errors.p(3,:), 'b', 'linewidth', 2); grid on;
ylabel('postion errors [m]')
xlabel('t [sec]');
legend('e_x', 'e_y', 'e_z');



% attitude ****************************************************************
figure(4)
subplot(3,1,1)
plot(Exp.odometry.t, -Exp.odometry.rpy(1,:)*rad2deg, 'r', 'linewidth', 2); grid on;
ylabel('roll [deg]');
subplot(3,1,2)
plot(Exp.odometry.t, Exp.odometry.rpy(2,:)*rad2deg, 'r', 'linewidth', 2); grid on;
ylabel('pitch [deg]')
subplot(3,1,3)
plot(Exp.odometry.t, Exp.odometry.rpy(3,:)*rad2deg, 'r', 'linewidth', 2); grid on;
ylabel('yaw [deg]')
xlabel('t [sec]');


% linear velocity
figure(5)
subplot(3,1,1)
plot(Exp.odometry.t, Exp.odometry.v(1,:), 'r', 'linewidth', 2); grid on;
xlabel('');
ylabel('vx [m/sec]');
subplot(3,1,2)
plot(Exp.odometry.t, Exp.odometry.v(2,:), 'g', 'linewidth', 2); grid on;
xlabel('');
ylabel('vy [m/sec]');
subplot(3,1,3)
plot(Exp.odometry.t, Exp.odometry.v(3,:), 'b', 'linewidth', 2); grid on;
xlabel('');
ylabel('vz [m/sec]');



% control inputs 
figure(6)
subplot(4,1,1)
plot(Exp.rpyt.t, -Exp.rpyt.roll*rad2deg, 'r', 'linewidth', 2); grid on;
ylabel('roll [deg]');
subplot(4,1,2)
plot(Exp.rpyt.t, Exp.rpyt.pitch*rad2deg, 'g', 'linewidth', 2); grid on;
ylabel('pitch [deg]');
subplot(4,1,3)
plot(Exp.rpyt.t, Exp.rpyt.yaw_rate*rad2deg, 'b', 'linewidth', 2); grid on;
ylabel('yaw-rate [deg/s]');
subplot(4,1,4)
plot(Exp.rpyt.t, Exp.rpyt.thrust(3,:), 'm', 'linewidth', 2); grid on;
ylabel('thrust [m/s]');
xlabel('t [sec]');



