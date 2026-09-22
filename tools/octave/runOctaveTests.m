function results = runOctaveTests(suites)
%RUNOCTAVETESTS Run this project's function-based test files under GNU Octave.
%
%   OCTAVE ONLY. results = RUNOCTAVETESTS(suites) runs each named test file
%   (a function returning functiontests(localfunctions)) and prints one
%   PASS/FAIL line per test plus a summary. Call setupOctave first.
%
%   This is a minimal stand-in for MATLAB's runtests: no fixtures, no
%   parameterisation; a verification failure stops that test.

if nargin < 1 || isempty(suites)
    suites = {'testGeometry','testCorridor','testPredictionAndRisk', ...
              'testPlannerCore','testDecisionAndVehicle','testClosedLoop', ...
              'testBehaviour'};
end
if ischar(suites), suites = {suites}; end

results = struct('suite', {}, 'name', {}, 'passed', {}, 'message', {}, 'seconds', {});
for s = 1:numel(suites)
    fprintf('\n--- %s ---\n', suites{s});
    try
        handles = feval(suites{s});
    catch err
        fprintf('  SUITE ERROR: %s\n', err.message);
        results(end+1) = struct('suite', suites{s}, 'name', '(load)', 'passed', false, ...
                                'message', err.message, 'seconds', 0); %#ok<AGROW>
        continue;
    end
    for i = 1:numel(handles)
        name = func2str(handles{i});
        if ~strncmpi(name, 'test', 4)
            continue;       % helper, not a test (same rule as MATLAB functiontests)
        end
        t0 = tic;
        try
            handles{i}(struct('suite', suites{s}));
            ok = true;  msg = '';
        catch err
            ok = false;
            msg = err.message;
            if ~isempty(err.stack)
                msg = sprintf('%s (%s line %d)', msg, err.stack(1).name, err.stack(1).line);
            end
        end
        dtt = toc(t0);
        if ok
            fprintf('  PASS  %-50s %6.1f s\n', name, dtt);
        else
            fprintf('  FAIL  %-50s %6.1f s\n        %s\n', name, dtt, msg);
        end
        results(end+1) = struct('suite', suites{s}, 'name', name, 'passed', ok, ...
                                'message', msg, 'seconds', dtt); %#ok<AGROW>
    end
end
nP = sum([results.passed]);
fprintf('\nOCTAVE RESULT: %d of %d passed, %d failed\n', nP, numel(results), numel(results) - nP);
end
