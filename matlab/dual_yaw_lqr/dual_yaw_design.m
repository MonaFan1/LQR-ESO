function p = dual_yaw_design(p)
%DUAL_YAW_DESIGN Rebuild the continuous plant and LQR from parameter fields.
%   State x = [q1; q1_dot; q2; q2_dot]. In 'article' mode q1 is the big stage
%   absolute angle and q2 the small stage RELATIVE encoder angle. In the
%   'screenshot'/'physical' variants both q1 and q2 are ABSOLUTE angles.
%   Inputs are [tau_big; tau_small], Nm, with reaction torque included.
%
%   P.A/P.B always describe the CONTINUOUS plant. For a discrete controller,
%   P.Ad/P.Bd are its zero-order-hold discretization at P.Ts. P.K gives
%   u_feedback = K*(x_ref - x) in either controller mode.

validateattributes(p, {'struct'}, {'scalar'}, mfilename, 'p');
p.model_mode = validatestring(p.model_mode, {'article', 'screenshot', 'physical'}, ...
    mfilename, 'p.model_mode');
p.controller_mode = validatestring(p.controller_mode, ...
    {'continuous', 'discrete'}, mfilename, 'p.controller_mode');
validateattributes(p.J1, {'numeric'}, {'real', 'finite', 'scalar', ...
    'positive'}, mfilename, 'p.J1');
validateattributes(p.J2, {'numeric'}, {'real', 'finite', 'scalar', ...
    'positive'}, mfilename, 'p.J2');
validateattributes(p.B1, {'numeric'}, {'real', 'finite', 'scalar', ...
    'nonnegative'}, mfilename, 'p.B1');
validateattributes(p.B2, {'numeric'}, {'real', 'finite', 'scalar', ...
    'nonnegative'}, mfilename, 'p.B2');
validateattributes(p.Ts, {'numeric'}, {'real', 'finite', 'scalar', ...
    'positive'}, mfilename, 'p.Ts');
validateattributes(p.Q, {'numeric'}, {'real', 'finite', 'size', [4, 4]}, ...
    mfilename, 'p.Q');
validateattributes(p.R, {'numeric'}, {'real', 'finite', 'size', [2, 2]}, ...
    mfilename, 'p.R');
check_positive_definite(p.Q, 'Q');
check_positive_definite(p.R, 'R');

switch p.model_mode
    case 'article'
        % q = [big absolute angle; small relative angle]. The article's
        % Lagrangian gives M*q_ddot = tau. Optional bearing damping extends
        % this to M*q_ddot + diag(B1,B2)*q_dot = tau.
        p.M = [p.J1+p.J2, p.J2; p.J2, p.J2];
        p.A = zeros(4);
        p.A(1, 2) = 1;
        p.A(3, 4) = 1;
        p.A([2, 4], [2, 4]) = -(p.M\diag([p.B1, p.B2]));
        p.B = zeros(4, 2);
        p.B([2, 4], :) = p.M\eye(2);
        p.coordinate_system = 'big_absolute_small_relative';
    case 'screenshot'
        % J1*a_big = tau_big - tau_small - B1*omega_big.
        % J2*a_small = tau_small - B2*omega_small.
        % This intentionally reproduces the diagram's simplified damping.
        p.A = [0, 1, 0, 0; ...
               0, -p.B1/p.J1, 0, 0; ...
               0, 0, 0, 1; ...
               0, 0, 0, -p.B2/p.J2];
        p.B = [0, 0; 1/p.J1, -1/p.J1; 0, 0; 0, 1/p.J2];
        p.M = diag([p.J1, p.J2]);
        p.coordinate_system = 'both_absolute';
    case 'physical'
        % Inner bearing friction is B2*(omega_small - omega_big); its
        % reaction on the big stage has the opposite sign.
        p.A = [0, 1, 0, 0; ...
               0, -(p.B1+p.B2)/p.J1, 0, p.B2/p.J1; ...
               0, 0, 0, 1; ...
               0, p.B2/p.J2, 0, -p.B2/p.J2];
        p.B = [0, 0; 1/p.J1, -1/p.J1; 0, 0; 0, 1/p.J2];
        p.M = diag([p.J1, p.J2]);
        p.coordinate_system = 'both_absolute';
end
p.C = eye(4);
p.D = zeros(4, 2);

if exist('lqr', 'file') == 0
    error('dual_yaw:ControlToolboxRequired', ...
        'Control System Toolbox is required for lqr/dlqr/c2d.');
end
p.controllability_rank = rank(ctrb(p.A, p.B));
if p.controllability_rank ~= 4
    error('dual_yaw:UncontrollablePlant', ...
        'The continuous plant is not fully controllable.');
end

p.Ad = [];
p.Bd = [];
switch p.controller_mode
    case 'continuous'
        [p.K, p.riccati_solution, p.closed_loop_poles] = ...
            lqr(p.A, p.B, p.Q, p.R);
        p.sample_time = 0;
        stable = all(real(p.closed_loop_poles) < 0);
    case 'discrete'
        sys_d = c2d(ss(p.A, p.B, p.C, p.D), p.Ts, 'zoh');
        p.Ad = sys_d.A;
        p.Bd = sys_d.B;
        if rank(ctrb(p.Ad, p.Bd)) ~= 4
            error('dual_yaw:UncontrollableSampledPlant', ...
                'The sampled plant is not fully controllable.');
        end
        [p.K, p.riccati_solution, p.closed_loop_poles] = ...
            dlqr(p.Ad, p.Bd, p.Q, p.R);
        p.sample_time = p.Ts;
        stable = all(abs(p.closed_loop_poles) < 1);
end
if ~stable
    error('dual_yaw:UnstableLinearClosedLoop', ...
        'LQR design did not produce a stable unsaturated linear system.');
end
end

function check_positive_definite(matrix, name)
% Reject asymmetric weights instead of silently using one matrix triangle.
if norm(matrix-matrix.', 'fro') > 1e-10*max(1, norm(matrix, 'fro'))
    error('dual_yaw:AsymmetricWeight', '%s must be symmetric.', name);
end
[~, flag] = chol(matrix);
if flag ~= 0
    error('dual_yaw:NonPositiveWeight', ...
        '%s must be positive definite.', name);
end
end
