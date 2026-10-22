function build_model
%BUILD_MODEL Create tank_control.slx using only MATLAB and Simulink.
% Run from a writable MATLAB Online folder. Replaces tank_control.slx.
% Model: A*dh/dt = q_in - k_out*sqrt(h) - disturbance.
% Positive disturbance is an additional withdrawal (m^3/s).
% All assumptions are editable in the model workspace variable p.

folder = fileparts(mfilename('fullpath'));
mdl = 'tank_control';
file = fullfile(folder, [mdl '.slx']);
if bdIsLoaded(mdl)
    if strcmp(get_param(mdl, 'Dirty'), 'on')
        error('Save or close the modified tank_control model before rebuilding.');
    end
    close_system(mdl, 0);
end
load_system('simulink');
new_system(mdl);
p = struct('A',0.05, 'k_out',0.0015, 'h0',0.4, ...
    'r0',0.4, 'r1',0.6, 't_ref',20, 't_dist',180, ...
    'dist_amp',0.0003, 'q_max',0.004, 'Kp',0.004, ...
    'Ki',0.00015, 'Kd',0.004, 'N',5, 'Kaw',0.2, ...
    'integral_enabled',1, 't_end',360);
p.q_bias = p.k_out*sqrt(p.h0);
w = get_param(mdl, 'ModelWorkspace');
assignin(w, 'p', p);
set_param(mdl, 'SolverType','Variable-step', 'Solver','ode45', ...
    'StopTime','p.t_end', 'MaxStep','0.1', 'RelTol','1e-6', ...
    'AbsTol','1e-8', 'ReturnWorkspaceOutputs','on', ...
    'SaveTime','off', 'SaveOutput','off', 'SignalLogging','off');

add('Sources/Step','Reference',[25 45 75 75], ...
    'Time','p.t_ref','Before','p.r0','After','p.r1');
add('Math Operations/Sum','Error',[115 45 140 75],'Inputs','+-');
add('Math Operations/Gain','P',[185 30 255 65],'Gain','p.Kp');
add('Math Operations/Gain','I gain',[185 100 255 135],'Gain','p.Ki');
add('Math Operations/Sum','I input',[300 105 325 140],'Inputs','++');
add('Math Operations/Gain','Enable I',[355 105 425 140], ...
    'Gain','p.integral_enabled');
add('Continuous/Integrator','I state',[460 105 495 140], ...
    'InitialCondition','0');
% Derivative of level deviation avoids both setpoint and startup kicks.
add('Sources/Constant','Initial level',[150 210 210 240],'Value','p.h0');
add('Math Operations/Sum','Level deviation',[265 205 290 240], ...
    'Inputs','+-');
add('Continuous/Transfer Fcn','D measurement',[350 205 475 245], ...
    'Numerator','[p.Kd*p.N 0]','Denominator','[1 p.N]');
add('Sources/Constant','Bias',[460 285 520 315],'Value','p.q_bias');
add('Math Operations/Sum','Command',[560 55 585 150],'Inputs','++-+');
add('Discontinuities/Saturation','Pump',[630 80 695 120], ...
    'LowerLimit','0','UpperLimit','p.q_max');
add('Math Operations/Sum','Saturation difference',[735 225 760 265], ...
    'Inputs','+-');
add('Math Operations/Gain','Anti windup',[555 350 645 385], ...
    'Gain','p.Kaw');
add('Sources/Step','Disturbance',[725 335 785 365], ...
    'Time','p.t_dist','Before','0','After','p.dist_amp');
add('Math Operations/Sum','Flow balance',[835 80 860 135], ...
    'Inputs','+--');
add('Math Operations/Gain','Inverse area',[900 85 970 120], ...
    'Gain','1/p.A');
% Lower-limited integrator prevents an unphysical negative tank level.
% The ideal tank has no upper wall or overflow; actuator inflow is limited.
add('Continuous/Integrator','Level',[1010 85 1050 120], ...
    'InitialCondition','p.h0','LimitOutput','on', ...
    'LowerSaturationLimit','0','UpperSaturationLimit','inf');
add('Math Operations/Sqrt','Square root',[1040 205 1100 240], ...
    'Operator','sqrt');
add('Math Operations/Gain','Outlet coefficient',[885 205 975 240], ...
    'Gain','p.k_out');
add('Signal Routing/Mux','Log signals',[1165 45 1170 185],'Inputs','5');
add('Sinks/To Workspace','Record',[1210 85 1310 125], ...
    'VariableName','tank_log','SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','Decimation','1');

wire('Reference/1','Error/1');
wire('Level/1','Error/2');
wire('Error/1','P/1');
wire('Error/1','I gain/1');
wire('I gain/1','I input/1');
wire('Anti windup/1','I input/2');
wire('I input/1','Enable I/1');
wire('Enable I/1','I state/1');
wire('P/1','Command/1');
wire('I state/1','Command/2');
wire('Level/1','Level deviation/1');
wire('Initial level/1','Level deviation/2');
wire('Level deviation/1','D measurement/1');
wire('D measurement/1','Command/3');
wire('Bias/1','Command/4');
wire('Command/1','Pump/1');
wire('Pump/1','Saturation difference/1');
wire('Command/1','Saturation difference/2');
wire('Saturation difference/1','Anti windup/1');
wire('Pump/1','Flow balance/1');
wire('Outlet coefficient/1','Flow balance/2');
wire('Disturbance/1','Flow balance/3');
wire('Flow balance/1','Inverse area/1');
wire('Inverse area/1','Level/1');
wire('Level/1','Square root/1');
wire('Square root/1','Outlet coefficient/1');
% Logged columns: reference, level, inlet, disturbance, natural outlet.
wire('Reference/1','Log signals/1');
wire('Level/1','Log signals/2');
wire('Pump/1','Log signals/3');
wire('Disturbance/1','Log signals/4');
wire('Outlet coefficient/1','Log signals/5');
wire('Log signals/1','Record/1');
set_param(mdl, 'SimulationCommand','update');
save_system(mdl, file);
open_system(mdl);
fprintf('Created %s\n', file);
fprintf('Parameters: Model Explorer > tank_control > Model Workspace > p.\n');

    function add(libraryPath, name, position, varargin)
        add_block(['simulink/' libraryPath], [mdl '/' name], ...
            'Position',position, varargin{:});
    end
    function wire(source, destination)
        add_line(mdl, source, destination, 'autorouting','on');
    end
end
