function run_experiments
%RUN_EXPERIMENTS Simulate P, PI and PID with MATLAB and Simulink only.
% Upload this file and build_model.m to the same writable Online folder.
% No tuning/optimization toolbox or helper file is required.
folder = fileparts(mfilename('fullpath'));
mdl = 'tank_control';
modelFile = fullfile(folder, [mdl '.slx']);
if ~isfile(modelFile)
    build_model;
end
if ~bdIsLoaded(mdl)
    load_system(modelFile);
end
w = get_param(mdl, 'ModelWorkspace');
p = getVariable(w, 'p');
assert(p.t_ref > 0 && p.t_dist > p.t_ref && p.t_end > p.t_dist, ...
    'Require 0 < t_ref < t_dist < t_end.');
assert(p.r1 > p.r0, 'These step-response metrics assume an upward setpoint step.');
names = {'P','PI','PID'};
% Fixed, illustrative gains, not claimed optimal. Same Kp for all cases.
gains = [p.Kp 0 0; p.Kp p.Ki 0; p.Kp p.Ki p.Kd];
results = struct;
metrics = zeros(3,10);
for k = 1:3
    pk = p;
    pk.Kp = gains(k,1);
    pk.Ki = gains(k,2);
    pk.Kd = gains(k,3);
    pk.integral_enabled = double(k > 1);
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('p', pk, 'Workspace', mdl);
    in = in.setModelParameter('ReturnWorkspaceOutputs','on');
    fprintf('Running %s ...\n', names{k});
    out = sim(in);
    log = out.get('tank_log');
    t = double(log.Time(:));
    y = double(log.Data);
    assert(size(y,1) == numel(t) && size(y,2) == 5, ...
        'Expected tank_log to contain five signal columns.');
    assert(all(isfinite(y(:))), 'Simulation returned nonfinite data.');
    % Resolve duplicate event timestamps by retaining the post-event sample.
    [t, idx] = unique(t, 'last');
    y = y(idx,:);
    results.(names{k}) = struct('time',t,'signals',y,'parameters',pk);
    e = y(:,1)-y(:,2);
    track = t >= p.t_ref & t < p.t_dist;
    disturb = t >= p.t_dist;
    preTail = t >= p.t_dist-0.1*(p.t_dist-p.t_ref) & t < p.t_dist;
    endTail = t >= p.t_end-0.1*(p.t_end-p.t_dist);
    tt = t(track); hh = y(track,2);
    td = t(disturb); hd = y(disturb,2);
    band = 0.02*abs(p.r1-p.r0);
    overshoot = max(0, max(hh)-p.r1)/abs(p.r1-p.r0)*100;
    low = firstCrossing(tt,hh,p.r0+0.1*(p.r1-p.r0));
    high = firstCrossing(tt,hh,p.r0+0.9*(p.r1-p.r0));
    rise = high-low;
    settle = settlingTime(tt,hh,p.r1,band,p.t_ref);
    recover = settlingTime(td,hd,p.r1,band,p.t_dist);
    metrics(k,:) = [trapz(t,abs(e)), trapz(t,e.^2), ...
        overshoot, rise, settle, mean(e(preTail)), ...
        max(abs(hd-p.r1)), recover, mean(e(endTail)), ...
        trapz(t,double(y(:,3)<=1e-9 | y(:,3)>=p.q_max-1e-9)) ...
        /(t(end)-t(1))*100];
end
summary = array2table(metrics, 'VariableNames', ...
    {'IAE_m_s','ISE_m2_s','TrackingOvershoot_pct','Rise10to90_s', ...
    'TrackingSettling_s','PreDistMeanError_m','DistMaxAbsError_m', ...
    'DistRecovery_s','FinalMeanError_m','PumpSaturated_pct'}, ...
    'RowNames',names);
outFolder = fullfile(folder,'results');
if ~isfolder(outFolder), mkdir(outFolder); end
writetable(summary, fullfile(outFolder,'metrics.csv'),'WriteRowNames',true);
save(fullfile(outFolder,'experiment_data.mat'),'results','summary','gains');
f = figure('Name','Nonlinear tank control comparison','Color','w');
colors = lines(3);
for panel = 1:3
    subplot(3,1,panel); hold on;
    for k = 1:3
        data = results.(names{k});
        if panel == 1, values = data.signals(:,2);
        elseif panel == 2, values = data.signals(:,3);
        else, values = data.signals(:,1)-data.signals(:,2);
        end
        plot(data.time,values,'Color',colors(k,:),'LineWidth',1.3, ...
            'DisplayName',names{k});
    end
    if panel == 1
        plot(data.time,data.signals(:,1),'k--','DisplayName','Reference');
        ylabel('Level (m)'); title('Nonlinear tank: P, PI and PID');
    elseif panel == 2
        plot(data.time,data.signals(:,4),'k--','DisplayName','Extra outflow');
        ylabel('Flow (m^3/s)'); title('Pump inflow and outflow disturbance');
    else
        ylabel('Error (m)'); title('Reference minus tank level');
    end
    xline(p.t_ref,':','Setpoint','HandleVisibility','off');
    xline(p.t_dist,':','Disturbance','HandleVisibility','off');
    xlabel('Time (s)'); grid on; legend('Location','best');
end
savefig(f, fullfile(outFolder,'comparison.fig'));
print(f, fullfile(outFolder,'comparison.png'),'-dpng','-r150');
disp(summary);
fprintf(['Metrics use a 2%% band of the setpoint step (%.4g m).\n' ...
    'Settling/recovery are elapsed from their respective events; NaN means\n' ...
    'the criterion was not reached within the observed window.\n' ...
    'Pre/final mean errors use the last 10%% of each window.\n' ...
    'DistMaxAbsError includes any error remaining before disturbance.\n' ...
    'IAE and ISE cover the full run; saturation percentage is time weighted.\n' ...
    'Saved outputs in %s\n'],band,outFolder);
end

function tCross = firstCrossing(t,h,threshold)
i = find(h >= threshold,1,'first');
if isempty(i), tCross = NaN; return; end
if i == 1, tCross = t(1); return; end
tCross = t(i-1)+(threshold-h(i-1))*(t(i)-t(i-1))/(h(i)-h(i-1));
end

function elapsed = settlingTime(t,h,target,band,eventTime)
% Last-exit criterion on logged samples, through the end of this window.
lastOutside = find(abs(h-target)>band,1,'last');
if isempty(lastOutside)
    elapsed = t(1)-eventTime;
elseif lastOutside == numel(t)
    elapsed = NaN;
else
    elapsed = t(lastOutside+1)-eventTime;
end
end
