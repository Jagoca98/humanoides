% teo_plantilla_m controller.

% A, l0, l1: all defined in fwdKin and invKin scopes at bottom

% create the Robot instance.

motor_q0 = wb_robot_get_device('r_shoulder_pitch');
motor_q1 = wb_robot_get_device('r_elbow_pitch');

% target 0

%% target 0: setPosition

% __3__
target0_q0 = ... % use helper: degtorad()
target0_q1 = ... % use helper: degtorad()
wb_motor_set_position(motor_q0, target0_q0);
wb_motor_set_position(motor_q1, target0_q1);
wb_robot_step(1000);
pause(1);

%% target 0: perform fwdKin

[target0_x, target0_z] = fwdKin(target0_q0, target0_q1);

%% target 0: check if invKin works to recover original joint space targets

[recovered0_q0, recovered0_q1] = invKin(target0_x, target0_z);

% target 1

%% target 1: generate in Cartesian space, adding offset from target 0

% __4a__
target1_x = ...
target1_z = ...

%% target 1: compute in joint space, via invKin

[target1_q0, target1_q1] = invKin(target1_x, target1_z);

% target 2

%% target 2: generate in Cartesian space

% __4b__
target2_x = ...
target2_z = ...

%% target 2: compute in joint space, via invKin

[target2_q0, target2_q1] = invKin(target2_x, target2_z);

% target 3

%% target 3: generate in Cartesian space, removing offset from target 0

% __4c__
target3_x = ...
target3_z = ...

%% target 3: compute in joint space, via invKin

[target3_q0, target3_q1] = invKin(target3_x, target3_z);

% Main loop:
% - perform simulation steps until Webots is stopping the controller
while true
  % __5__
  wb_motor_set_position(motor_q0, target1_q0);
  wb_motor_set_position(motor_q1, target1_q1);
  if -1 == wb_robot_step(1000);
    break;
  end
  pause(1);
  wb_motor_set_position(motor_q0, target2_q0);
  wb_motor_set_position(motor_q1, target2_q1);
  if -1 == wb_robot_step(1000);
    break;
  end
  pause(1);
  wb_motor_set_position(motor_q0, target3_q0);
  wb_motor_set_position(motor_q1, target3_q1);
  if -1 == wb_robot_step(1000);
    break;
  end
  pause(1);
  wb_motor_set_position(motor_q0, target2_q0);
  wb_motor_set_position(motor_q1, target2_q1);
  if -1 == wb_robot_step(1000);
    break;
  end
  pause(1);
end

function [x, z] = fwdKin(q0, q1)
  % __1__
  l0 = 0.329;
  l1 = 0.215+0.090;
  A = l0+l1;
  wb_console_print(['fwdKin: input (q1 q2): ' num2str(q0) ' ' num2str(q1)], WB_STDOUT);
  u = ...
  v = ...
  wb_console_print(['fwdKin: intermediate (u v): ' num2str(u) ' ' num2str(v)], WB_STDOUT);
  x = ...
  z = ...
  wb_console_print(['fwdKin: output (x z): ' num2str(x) ' ' num2str(z)], WB_STDOUT);
end

function [q0, q1] = invKin(x, z)
  % __2__
  l0 = 0.329;
  l1 = 0.215+0.090;
  A = l0+l1;
  wb_console_print(['invKin: input (x z): ' num2str(x) ' ' num2str(z)], WB_STDOUT);
  u = ...
  v = ...
  wb_console_print(['invKin: intermediate (u v): ' num2str(u) ' ' num2str(v)], WB_STDOUT);
  q1 = ...
  q0 = ...
  wb_console_print(['invKin: output (q0 q1): ' num2str(radtodeg(q0)) ' ' num2str(radtodeg(q1))], WB_STDOUT);
end