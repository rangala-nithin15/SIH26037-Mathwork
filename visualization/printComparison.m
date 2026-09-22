function printComparison(results)
%PRINTCOMPARISON Print the baseline-versus-IR-PSC comparison table.
%
%   COMPONENT STATUS: REAL
%
%   PRINTCOMPARISON(results) prints the output of RUNCOMPARISON as a table.
%
%   Every number printed comes from the results struct. Nothing is
%   hard-coded, and the function refuses to print anything if no runs were
%   performed. The provenance footer is printed with every table so a
%   screenshot of the results can never be separated from the caveat that
%   they are simulation results.
%
%   Inputs:
%       results - struct from runComparison()
%
%   Example:
%       r = runComparison({'village'}, 1:5);
%       printComparison(r);
%
%   Requires: base MATLAB only.
%
%   See also RUNCOMPARISON, RUNABLATION.

if ~isstruct(results) || ~isfield(results,'summary')
    error('printComparison:noResults', ...
          'No results supplied. Run runComparison() first -- this project never prints fabricated results.');
end

S = results.summary;
n = numel(S.scenario);

fprintf('\n');
fprintf('=====================================================================\n');
fprintf('  BASELINE vs IR-PSC\n');
fprintf('  Runs: %d   Seeds: %s   Generated: %s\n', ...
        results.nRuns, mat2str(results.seeds), results.timestamp);
fprintf('=====================================================================\n\n');

fprintf('%-12s %-22s %10s %10s\n', 'Scenario', 'Metric', 'Baseline', 'IR-PSC');
fprintf('%s\n', repmat('-', 1, 58));

for i = 1:n
    printRow(S.scenario{i}, 'worst clearance (m)', ...
             S.baselineMinClearance(i), S.irpscMinClearance(i));
    printRow('', 'collisions (mean)', ...
             S.baselineCollisions(i), S.irpscCollisions(i));
    printRow('', 'completion rate', ...
             S.baselineCompletion(i), S.irpscCompletion(i));
    printRow('', 'avg speed (m/s)', ...
             S.baselineAvgSpeed(i), S.irpscAvgSpeed(i));
    printRow('', 'smoothness (lower=better)', ...
             S.baselineSmoothness(i), S.irpscSmoothness(i));
    printRow('', 'planner failures (mean)', ...
             S.baselineFailures(i), S.irpscFailures(i));
    fprintf('%s\n', repmat('-', 1, 58));
end

fprintf('\n');
fprintf('HOW TO READ THIS TABLE\n');
fprintf('  Higher is better: clearance, completion rate, average speed.\n');
fprintf('  Lower is better:  collisions, smoothness, planner failures.\n');
fprintf('  Clearance is the WORST case across runs, not the mean, because\n');
fprintf('  the mean of a minimum is not a safety statement.\n');
fprintf('\n');
fprintf('PROVENANCE AND LIMITS\n');
fprintf('  Simulation results only, under a SIMPLIFIED geometric sensor\n');
fprintf('  model and a SIMPLIFIED kinematic vehicle model. Valid for\n');
fprintf('  comparing planners under identical conditions. NOT real-world\n');
fprintf('  safety evidence. A collision count of zero here does NOT mean\n');
fprintf('  collisions are avoided in reality, and this system is not\n');
fprintf('  safety certified.\n');
fprintf('=====================================================================\n\n');
end

% =====================================================================
function printRow(scenario, metric, baselineVal, irpscVal)
fprintf('%-12s %-22s %10s %10s\n', scenario, metric, ...
        fmt(baselineVal), fmt(irpscVal));
end

% =====================================================================
function s = fmt(v)
if isnan(v)
    s = 'n/a';
elseif abs(v) >= 1000 || (abs(v) < 0.01 && v ~= 0)
    s = sprintf('%.2e', v);
else
    s = sprintf('%.3f', v);
end
end
