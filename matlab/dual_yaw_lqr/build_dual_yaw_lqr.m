function model = build_dual_yaw_lqr(p)
%BUILD_DUAL_YAW_LQR Create the editable, coupled two-yaw Simulink model.
% All angles are in rad, speeds in rad/s and torques in N*m.
if nargin < 1
    p = dual_yaw_parameters();
end
folder = fileparts(mfilename('fullpath'));
model = 'dual_yaw_lqr';
model_file = fullfile(folder, [model '.slx']);
if bdIsLoaded(model)
    error('dual_yaw:ModelOpen', ...
        'Close dual_yaw_lqr before rebuilding; save any manual block edits first.');
end
load_system('simulink');
new_system(model);
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', 'p.solver_step', 'StopTime', 'p.stop_time', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'on', ...
    'SaveOutput', 'off', 'SignalLogging', 'off');

% Reference -> state error -> full 2-by-4 LQR -> per-motor limits.
block('simulink/Sources/From Workspace', 'Reference', [35 90 165 130], ...
    'VariableName', 'x_ref_input', 'Interpolate', 'on');
block('simulink/Math Operations/Sum', 'StateError', [225 90 250 130], ...
    'Inputs', '+-', 'IconShape', 'rectangular');
block('simulink/Math Operations/Gain', 'FeedbackSampling', ...
    [295 95 370 125], 'Gain', '1');
block('simulink/Math Operations/Gain', 'LQR_K', [420 85 505 135], ...
    'Gain', 'p.K', 'Multiplication', 'Matrix(K*u)');
block('simulink/Sources/From Workspace', 'Feedforward', [275 190 405 230], ...
    'VariableName', 'u_ff_input', 'Interpolate', 'on');
block('simulink/Math Operations/Gain', 'FeedforwardSampling', ...
    [440 195 515 225], 'Gain', '1');
block('simulink/Math Operations/Sum', 'TorqueCommand', [565 90 590 130], ...
    'Inputs', '++', 'IconShape', 'rectangular');
block('simulink/Signal Routing/Demux', 'MotorCommands', [630 87 635 273], ...
    'Outputs', '2');
block('simulink/Discontinuities/Dead Zone', 'DM4310_DeadZone', ...
    [685 105 760 140], 'LowerValue', '-p.dead_zone(1)', ...
    'UpperValue', 'p.dead_zone(1)');
block('simulink/Discontinuities/Saturation', 'DM4310_Limit', ...
    [795 105 880 140], 'LowerLimit', '-p.torque_limit(1)', ...
    'UpperLimit', 'p.torque_limit(1)');
block('simulink/Discontinuities/Dead Zone', 'GM6020_DeadZone', ...
    [685 220 760 255], 'LowerValue', '-p.dead_zone(2)', ...
    'UpperValue', 'p.dead_zone(2)');
block('simulink/Discontinuities/Saturation', 'GM6020_Limit', ...
    [795 220 880 255], 'LowerLimit', '-p.torque_limit(2)', ...
    'UpperLimit', 'p.torque_limit(2)');
block('simulink/Signal Routing/Mux', 'ActualTorques', [920 115 925 245], ...
    'Inputs', '2');

% Physical plant: acceleration = B_acc*tau + A_vel*velocity.
% Matrix B_acc retains the reaction of the small motor on the large motor.
block('simulink/Math Operations/Gain', 'CoupledInertia', ...
    [990 135 1100 180], 'Gain', 'p.B([2 4],:)', ...
    'Multiplication', 'Matrix(K*u)');
block('simulink/Math Operations/Sum', 'Acceleration', [1150 140 1175 180], ...
    'Inputs', '++', 'IconShape', 'rectangular');
block('simulink/Math Operations/Gain', 'ViscousDynamics', ...
    [995 345 1105 395], 'Gain', 'p.A([2 4],[2 4])', ...
    'Multiplication', 'Matrix(K*u)');
block('simulink/Signal Routing/Demux', 'AxisAcceleration', ...
    [1210 130 1215 285], 'Outputs', '2');
block('simulink/Continuous/Integrator', 'LargeYawSpeed', ...
    [1260 145 1295 180], 'InitialCondition', 'p.x0(2)');
block('simulink/Continuous/Integrator', 'LargeYawAngle', ...
    [1350 145 1385 180], 'InitialCondition', 'p.x0(1)');
block('simulink/Continuous/Integrator', 'SmallYawSpeed', ...
    [1260 260 1295 295], 'InitialCondition', 'p.x0(4)');
block('simulink/Continuous/Integrator', 'SmallYawAngle', ...
    [1350 260 1385 295], 'InitialCondition', 'p.x0(3)');
block('simulink/Signal Routing/Mux', 'Velocities', [1450 350 1455 410], ...
    'Inputs', '2');
block('simulink/Signal Routing/Mux', 'State', [1490 150 1495 300], ...
    'Inputs', '4');

% Visible scopes and exported time series.
block('simulink/Sinks/Scope', 'TrackingScope', [1605 80 1660 145], ...
    'NumInputPorts', '2');
block('simulink/Sinks/Scope', 'TorqueScope', [1010 455 1065 500]);
block('simulink/Sinks/To Workspace', 'LogState', [1590 220 1705 255], ...
    'VariableName', 'state_log', 'SaveFormat', 'Timeseries');
block('simulink/Sinks/To Workspace', 'LogReference', [95 455 220 490], ...
    'VariableName', 'reference_log', 'SaveFormat', 'Timeseries');
block('simulink/Sinks/To Workspace', 'LogError', [280 455 405 490], ...
    'VariableName', 'error_log', 'SaveFormat', 'Timeseries');
block('simulink/Sinks/To Workspace', 'LogTorque', [800 455 925 490], ...
    'VariableName', 'torque_log', 'SaveFormat', 'Timeseries');
block('simulink/Sinks/To Workspace', 'LogCommand', [585 455 710 490], ...
    'VariableName', 'command_log', 'SaveFormat', 'Timeseries');

wire('Reference/1', 'StateError/1');
wire('State/1', 'StateError/2');
wire('StateError/1', 'FeedbackSampling/1');
wire('FeedbackSampling/1', 'LQR_K/1');
wire('LQR_K/1', 'TorqueCommand/1');
wire('Feedforward/1', 'FeedforwardSampling/1');
wire('FeedforwardSampling/1', 'TorqueCommand/2');
wire('TorqueCommand/1', 'MotorCommands/1');
wire('MotorCommands/1', 'DM4310_DeadZone/1');
wire('DM4310_DeadZone/1', 'DM4310_Limit/1');
wire('MotorCommands/2', 'GM6020_DeadZone/1');
wire('GM6020_DeadZone/1', 'GM6020_Limit/1');
wire('DM4310_Limit/1', 'ActualTorques/1');
wire('GM6020_Limit/1', 'ActualTorques/2');
wire('ActualTorques/1', 'CoupledInertia/1');
wire('CoupledInertia/1', 'Acceleration/1');
wire('ViscousDynamics/1', 'Acceleration/2');
wire('Acceleration/1', 'AxisAcceleration/1');
wire('AxisAcceleration/1', 'LargeYawSpeed/1');
wire('LargeYawSpeed/1', 'LargeYawAngle/1');
wire('AxisAcceleration/2', 'SmallYawSpeed/1');
wire('SmallYawSpeed/1', 'SmallYawAngle/1');
wire('LargeYawSpeed/1', 'Velocities/1');
wire('SmallYawSpeed/1', 'Velocities/2');
wire('Velocities/1', 'ViscousDynamics/1');
wire('LargeYawAngle/1', 'State/1');
wire('LargeYawSpeed/1', 'State/2');
wire('SmallYawAngle/1', 'State/3');
wire('SmallYawSpeed/1', 'State/4');
wire('Reference/1', 'TrackingScope/1');
wire('State/1', 'TrackingScope/2');
wire('State/1', 'LogState/1');
wire('Reference/1', 'LogReference/1');
wire('StateError/1', 'LogError/1');
wire('TorqueCommand/1', 'LogCommand/1');
wire('ActualTorques/1', 'LogTorque/1');
wire('ActualTorques/1', 'TorqueScope/1');

set_param([model '/Reference'], 'BackgroundColor', '[0.83,0.95,0.83]');
set_param([model '/Feedforward'], 'BackgroundColor', '[0.83,0.95,0.83]');
set_param([model '/LQR_K'], 'BackgroundColor', '[0.78,0.88,1]');
set_param([model '/DM4310_Limit'], 'BackgroundColor', '[1,0.86,0.70]');
set_param([model '/GM6020_Limit'], 'BackgroundColor', '[1,0.86,0.70]');
set_param([model '/CoupledInertia'], 'BackgroundColor', '[0.92,0.85,1]');
set_param([model '/ViscousDynamics'], 'BackgroundColor', '[0.92,0.85,1]');

note = Simulink.Annotation(model, sprintf([ ...
    'COAXIAL DUAL-YAW LQR | DM4310 + GM6020\n' ...
    'Default: article relative coordinates x = [theta_big; omega_big; theta_small_relative; omega_small_relative]\n' ...
    'u = sat( K*(x_ref - x) + u_ff ) | J values: article CAD estimates; Q/R: demo tuning\n' ...
    'CoupledInertia includes small-motor reaction. Change parameters with run_dual_yaw_lqr(p).']));
note.Position = [35 530 1580 610];
note.FontSize = 12;
dual_yaw_configure(model, p);
set_param(model, 'Location', [30 60 1770 780], 'ZoomFactor', 'FitSystem');
save_system(model, model_file);
fprintf('Created %s\n', model_file);

    function block(library, name, position, varargin)
        add_block(library, [model '/' name], 'Position', position, varargin{:});
    end
    function wire(source, destination)
        add_line(model, source, destination, 'autorouting', 'on');
    end
end
