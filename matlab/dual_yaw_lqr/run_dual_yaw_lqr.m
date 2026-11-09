function result = run_dual_yaw_lqr(p, show_windows)
%RUN_DUAL_YAW_LQR Build if necessary, simulate, plot, and save the results.
%   run_dual_yaw_lqr
%   p = dual_yaw_parameters(); p.Q(3,3) = 400; run_dual_yaw_lqr(p);
if nargin < 1
    p = dual_yaw_parameters();
end
if nargin < 2
    show_windows = true;
end
folder = fileparts(mfilename('fullpath'));
model = 'dual_yaw_lqr';
file = fullfile(folder, [model '.slx']);
if ~isfile(file)
    build_dual_yaw_lqr(p);
else
    load_system(file);
end
p = dual_yaw_configure(model, p);
save_system(model, file);
out = sim(model);
result.p = p;
result.time = out.state_log.Time;
result.state = out.state_log.Data;
result.reference = out.reference_log.Data;
result.error = out.error_log.Data;
result.torque = out.torque_log.Data;
result.command = out.command_log.Data;
assert(all(isfinite(result.state), 'all'), 'Non-finite plant states.');
assert(all(abs(result.torque) <= p.torque_limit' + 1e-9, 'all'), ...
    'A motor exceeded its configured torque limit.');
result.angle_rms_deg = rad2deg(sqrt(mean(result.error(:, [1 3]).^2, 1)));
result.peak_torque = max(abs(result.torque), [], 1);
save(fullfile(folder, 'dual_yaw_result.mat'), 'result');

visibility = 'off';
if show_windows
    visibility = 'on';
    open_system(model);
end
fig = figure('Name', 'Dual-yaw LQR', 'Color', 'w', ...
    'Visible', visibility, 'Position', [80 60 1200 800]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact');
nexttile;
plot(result.time, rad2deg(result.reference(:,1)), '--', ...
    result.time, rad2deg(result.state(:,1)), 'LineWidth', 1.1);
grid on; title('Large yaw'); ylabel('angle [deg]'); legend('reference', 'actual');
nexttile;
plot(result.time, rad2deg(result.reference(:,3)), '--', ...
    result.time, rad2deg(result.state(:,3)), 'LineWidth', 1.1);
grid on;
if strcmp(p.model_mode, 'article')
    title('Small yaw RELATIVE to large yaw');
else
    title('Small yaw ABSOLUTE angle');
end
ylabel('angle [deg]'); legend('reference', 'actual');
nexttile;
plot(result.time, rad2deg(result.error(:,[1 3])), 'LineWidth', 1.1);
grid on; title('Angle error'); ylabel('error [deg]'); xlabel('time [s]');
legend('large yaw', 'small yaw');
nexttile;
plot(result.time, result.torque, 'LineWidth', 1.1);
hold on;
yline(p.torque_limit(1), ':'); yline(-p.torque_limit(1), ':');
yline(p.torque_limit(2), ':'); yline(-p.torque_limit(2), ':');
grid on; title('Actual limited motor torques'); ylabel('torque [N m]');
xlabel('time [s]'); legend('DM4310', 'GM6020');
title(layout, sprintf('%s | %s LQR | demonstration tuning', ...
    p.model_mode, p.controller_mode));
exportgraphics(fig, fullfile(folder, 'dual_yaw_response.png'), 'Resolution', 140);
print(['-s' model], '-dpng', '-r130', fullfile(folder, 'dual_yaw_model.png'));
if ~show_windows
    close(fig);
end
fprintf('K, columns [theta1 omega1 theta2 omega2]:\n'); disp(p.K);
fprintf('Angle RMS [large, small] = [%.4f %.4f] deg\n', result.angle_rms_deg);
fprintf('Peak torque [large, small] = [%.4f %.4f] N*m\n', result.peak_torque);
fprintf('Saved model, plots and results to %s\n', folder);
end
