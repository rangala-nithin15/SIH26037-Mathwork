function proj = createProject(projectName)
%CREATEPROJECT Create the MATLAB Project (.prj) on the destination machine.
%
%   COMPONENT STATUS: REAL SOURCE / EXECUTION **NOT VERIFIED**
%
%   ============================================================
%   WHY NO .prj FILE SHIPS WITH THIS PACKAGE
%   ============================================================
%   A MATLAB Project is not a single text file. It is a `.prj` file plus a
%   `resources/` folder of metadata that MATLAB generates and manages. Only
%   MATLAB can create it correctly.
%
%   MATLAB was not installed on the machine where this project was written,
%   so no `.prj` could be created or verified. Hand-writing one would have
%   produced a file that either fails to open or opens into a broken state.
%   This script generates a real one instead, on a machine that can.
%
%   THE PROJECT WORKS WITHOUT THIS. `setupPaths.m` adds every source folder
%   to the MATLAB path and is all that is strictly required. The MATLAB
%   Project adds convenience -- automatic path management, dependency
%   analysis, integrated source control -- but nothing depends on it.
%
%   proj = CREATEPROJECT(projectName) creates the project, adds every
%   source folder, and registers setupPaths as the startup file.
%
%   Inputs:
%       projectName - (optional) char. Default 'SIH_Indian_AV'.
%
%   Outputs:
%       proj - the matlab.project.Project object
%
%   Example:
%       cd <project root>
%       setupPaths;
%       createProject;
%
%   Requires: MATLAB R2019a or later. NOT VERIFIED on any machine.
%
%   See also SETUPPATHS.

if nargin < 1 || isempty(projectName)
    projectName = 'SIH_Indian_AV';
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));

% --- Guard: do not silently clobber an existing project -----------------
existing = dir(fullfile(projectRoot, '*.prj'));
if ~isempty(existing)
    error('createProject:alreadyExists', ...
          ['A project file already exists here: %s\n' ...
           'Open it instead, or delete it first if you intend to recreate it.'], ...
          existing(1).name);
end

if exist('matlab.project.createProject', 'file') ~= 2 && ...
   exist('matlab.project.createProject', 'builtin') == 0
    error('createProject:unsupportedRelease', ...
          ['matlab.project.createProject is not available on this MATLAB ' ...
           'release (needs R2019a or later). Use setupPaths.m instead -- ' ...
           'the project works without a .prj file.']);
end

fprintf('Creating MATLAB Project "%s" in:\n  %s\n\n', projectName, projectRoot);

proj = matlab.project.createProject('Name', projectName, 'Folder', projectRoot);

% --- Add source folders to the project path -----------------------------
pathFolders = { ...
    'config', 'utils', 'planner', ...
    fullfile('planner','IR_PSC'), ...
    fullfile('planner','IR_PSC','corridor'), ...
    fullfile('planner','IR_PSC','risk'), ...
    fullfile('planner','IR_PSC','deformation'), ...
    fullfile('planner','IR_PSC','safety'), ...
    fullfile('planner','IR_PSC','trajectory'), ...
    fullfile('planner','baseline'), ...
    'prediction', ...
    fullfile('prediction','baseline'), ...
    fullfile('prediction','uncertainty'), ...
    fullfile('prediction','irregular_motion'), ...
    'perception', ...
    fullfile('perception','detection'), ...
    fullfile('perception','tracking'), ...
    fullfile('perception','fusion'), ...
    'sensors', 'decision', ...
    fullfile('vehicle','controller'), ...
    fullfile('vehicle','dynamics'), ...
    'metrics', 'visualization', 'scripts', 'experiments', 'scenarios', ...
    fullfile('scenarios','village'), ...
    fullfile('scenarios','urban_intersection'), ...
    fullfile('scenarios','highway_merge'), ...
    fullfile('scenarios','market'), ...
    fullfile('scenarios','cattle_crossing'), ...
    fullfile('tests','unit'), ...
    fullfile('tests','integration') };

nAdded = 0;
for k = 1:numel(pathFolders)
    f = fullfile(projectRoot, pathFolders{k});
    if exist(f, 'dir') == 7
        try
            addPath(proj, f);
            nAdded = nAdded + 1;
        catch ME
            warning('createProject:addPathFailed', ...
                    'Could not add %s to the project path: %s', ...
                    pathFolders{k}, ME.message);
        end
    end
end
fprintf('Added %d folders to the project path.\n', nAdded);

% --- Register setupPaths as the startup file ----------------------------
try
    addStartupFile(proj, fullfile(projectRoot, 'setupPaths.m'));
    fprintf('Registered setupPaths.m as the project startup file.\n');
catch ME
    warning('createProject:startupFailed', ...
            'Could not register the startup file: %s', ME.message);
end

% --- Exclude generated output from the project --------------------------
% results/ holds generated figures and metrics. They should never be
% committed, and they should not clutter dependency analysis.
try
    if exist(fullfile(projectRoot,'results'), 'dir') == 7
        addPath(proj, fullfile(projectRoot,'results'));
        removePath(proj, fullfile(projectRoot,'results'));
    end
catch
    % Non-fatal: this is tidiness, not correctness.
end

fprintf('\nProject created. Next steps:\n');
fprintf('  1. runAllTests(''unit'')   -- expect some failures on first run\n');
fprintf('  2. demoScenario(''village'') -- see it move\n\n');
end
