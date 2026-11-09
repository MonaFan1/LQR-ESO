function p = dual_yaw_configure(model, p)
%DUAL_YAW_CONFIGURE Recompute gains and store all inputs inside the SLX file.
p = dual_yaw_design(p);
if ~isfield(p, 'dead_zone')
    p.dead_zone = [0; 0]; % Unknown in the article; disabled by default.
end
assert(all(p.dead_zone >= 0) && all(p.dead_zone < p.torque_limit));
t = (0:p.solver_step:p.stop_time)';
[x_ref, u_ff] = dual_yaw_reference(p, t);
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'p', p);
assignin(workspace, 'x_ref_input', timeseries(x_ref, t));
assignin(workspace, 'u_ff_input', timeseries(u_ff, t));
sample_blocks = {'FeedbackSampling', 'FeedforwardSampling'};
for index = 1:numel(sample_blocks)
    path = [model '/' sample_blocks{index}];
    if strcmp(p.controller_mode, 'discrete')
        desired = 'ZeroOrderHold';
        library = 'simulink/Discrete/Zero-Order Hold';
    else
        desired = 'Gain';
        library = 'simulink/Math Operations/Gain';
    end
    if ~strcmp(get_param(path, 'BlockType'), desired)
        % Port order stays identical, so existing lines reconnect automatically.
        position = get_param(path, 'Position');
        replace_block(model, 'SearchDepth', 1, 'Name', ...
            sample_blocks{index}, library, 'noprompt');
        set_param(path, 'Position', position);
    end
    if strcmp(p.controller_mode, 'discrete')
        set_param(path, 'SampleTime', 'p.Ts');
    else
        set_param(path, 'Gain', '1');
    end
end
end
