function validate_dual_yaw_lqr()
%VALIDATE_DUAL_YAW_LQR Check wiring against exact state-space solutions.
folder = fileparts(mfilename('fullpath'));
model = 'dual_yaw_lqr';
file = fullfile(folder, [model '.slx']);
if ~isfile(file)
    build_dual_yaw_lqr();
end
load_system(file);
restore = onCleanup(@() restore_default(model));

% A change of coordinates must preserve the same mechanical system.
physical = dual_yaw_parameters('physical');
relative = dual_yaw_parameters('article');
physical.B1 = 0.04; physical.B2 = 0.01;
relative.B1 = physical.B1; relative.B2 = physical.B2;
physical = dual_yaw_design(physical);
relative = dual_yaw_design(relative);
transform = [1 0 0 0; 0 1 0 0; -1 0 1 0; 0 -1 0 1];
assert(norm(relative.A - transform*physical.A/transform, 'fro') < 1e-10);
assert(norm(relative.B - transform*physical.B, 'fro') < 1e-10);
fprintf('PASS: absolute/relative coordinate equivalence.\n');

for mode = {'article', 'screenshot', 'physical'}
    p = dual_yaw_parameters(mode{1});
    p.stop_time = 0.25;
    p.B1 = 0.04; p.B2 = 0.01;
    p.x0 = [0.01; 0.1; -0.02; -0.08];
    p = dual_yaw_configure(model, p);
    assert(all(real(eig(p.A - p.B*p.K)) < 0));
    % Open-loop constant torques exercise reaction torque and damping signs.
    p.K = zeros(2,4);
    t = (0:p.solver_step:p.stop_time)';
    torques = repmat([0.2 0.08], numel(t), 1);
    workspace = get_param(model, 'ModelWorkspace');
    assignin(workspace, 'p', p);
    assignin(workspace, 'x_ref_input', timeseries(zeros(numel(t),4), t));
    assignin(workspace, 'u_ff_input', timeseries(torques, t));
    out = sim(model);
    expected = lsim(ss(p.A, p.B, eye(4), zeros(4,2)), torques, t, p.x0);
    difference = max(abs(out.state_log.Data - expected), [], 'all');
    assert(difference < 1e-6, 'Plant mismatch in %s: %.3g', mode{1}, difference);
    fprintf('PASS: %s integrator plant vs exact state space, error %.3g.\n', ...
        mode{1}, difference);
end

% Zero-reference recovery and clipping, both continuous and sampled LQR.
for controller = {'continuous', 'discrete'}
    p = dual_yaw_parameters('article', controller{1});
    p.stop_time = 4;
    p.x0 = [0.3; 0; -0.2; 0];
    p = dual_yaw_configure(model, p);
    t = (0:p.solver_step:p.stop_time)';
    workspace = get_param(model, 'ModelWorkspace');
    assignin(workspace, 'x_ref_input', timeseries(zeros(numel(t),4), t));
    assignin(workspace, 'u_ff_input', timeseries(zeros(numel(t),2), t));
    out = sim(model);
    assert(max(abs(out.state_log.Data(end,:))) < 1e-5);
    assert(all(abs(out.torque_log.Data) <= p.torque_limit' + 1e-9, 'all'));
    assert(any(abs(out.command_log.Data) > p.torque_limit', 'all'), ...
        'Recovery case did not exercise torque saturation.');
    if strcmp(controller{1}, 'discrete')
        assert(all(abs(p.closed_loop_poles) < 1));
    else
        assert(all(real(p.closed_loop_poles) < 0));
    end
    fprintf('PASS: %s closed-loop recovery, poles and torque limits.\n', controller{1});
end

% Smooth reference derivatives are numerical cross-checked away from endpoints.
p = dual_yaw_parameters();
p.ref_kind = 'chirp';
p.enable_feedforward = true;
t = (0:1e-5:3)';
[reference, ff] = dual_yaw_reference(p, t);
for index = [1 3]
    numeric_speed = gradient(reference(:,index), 1e-5);
    assert(max(abs(numeric_speed(5:end-5) - reference(5:end-5,index+1))) < 1e-4);
end
assert(all(isfinite(ff), 'all'));
fprintf('PASS: chirp analytic velocities and finite inverse-dynamics feedforward.\n');
clear restore;
fprintf('All dual-yaw model checks passed. Default model restored.\n');
end

function restore_default(model)
dual_yaw_configure(model, dual_yaw_parameters());
save_system(model);
end
