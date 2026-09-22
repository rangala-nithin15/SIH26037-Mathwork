function nCases = exportFrenetReference(outFile, nCases, seed)
%EXPORTFRENETREFERENCE Export Frenet test cases for the Python cross-check.
%
%   COMPONENT STATUS: REAL
%
%   nCases = EXPORTFRENETREFERENCE(outFile, nCases, seed) generates random
%   path/point pairs, computes their road-aligned (s, d) coordinates with
%   the MATLAB implementation, and writes them to CSV for
%   python/crosscheck_frenet.py to check independently.
%
%   Why bother
%   ----------
%   The (s, d) frame is the coordinate system the entire IR-PSC planner
%   operates in. A sign error in the lateral offset would make the planner
%   deform toward hazards instead of away from them -- and it would do so
%   consistently enough to look deliberate rather than broken. Checking the
%   transform against a second, independently written implementation is
%   cheap insurance against exactly that class of bug.
%
%   The cases deliberately include the awkward ones: points beyond the ends
%   of the path (which must clamp), points exactly on the path (d = 0),
%   points on both sides (to pin the sign convention), and paths with
%   near-duplicate points.
%
%   Inputs:
%       outFile - char, output CSV path. Default 'python/frenet_reference.csv'
%                 relative to the project root.
%       nCases  - number of random cases. Default 200.
%       seed    - RNG seed for reproducibility. Default 12345.
%
%   Outputs:
%       nCases - number of cases actually written
%
%   CSV format, one row per case:
%       n_path, px1,py1, px2,py2, ..., qx, qy, s, d
%
%   Example:
%       setupPaths;
%       exportFrenetReference('python/frenet_reference.csv');
%
%   Then, in a terminal:
%       python python/crosscheck_frenet.py python/frenet_reference.csv
%
%   Requires: base MATLAB only.
%
%   See also PROJECTPOINTONPATH, FRENETTOCARTESIAN.

if nargin < 1 || isempty(outFile)
    outFile = fullfile('python', 'frenet_reference.csv');
end
if nargin < 2 || isempty(nCases), nCases = 200;   end
if nargin < 3 || isempty(seed),   seed   = 12345; end

s = RandStream('mt19937ar', 'Seed', seed);

% Resolve relative paths against the project root, so this works from any
% working directory.
if ~isAbsolutePath(outFile)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    outFile = fullfile(projectRoot, outFile);
end

outDir = fileparts(outFile);
if ~isempty(outDir) && exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

fid = fopen(outFile, 'w');
if fid < 0
    error('exportFrenetReference:cannotWrite', 'Could not open %s', outFile);
end
cleaner = onCleanup(@() fclose(fid));

fprintf(fid, 'n_path,path_coords...,query_x,query_y,s,d\n');

written = 0;
for k = 1:nCases
    % --- Build a path -------------------------------------------------
    n = 3 + randi(s, 8);
    switch mod(k, 4)
        case 0   % straight along x: the simplest sign check
            P = [(0:n-1).' * 3, zeros(n,1)];
        case 1   % arc
            th = linspace(0, pi/3, n).';
            P  = [12*cos(th), 12*sin(th)];
        case 2   % random walk
            P = cumsum([0 0; randn(s, n-1, 2) * 3], 1);
        otherwise % path containing a near-duplicate point, which breaks
                  % naive implementations that divide by segment length
            P = cumsum([0 0; randn(s, n-2, 2) * 3], 1);
            P = [P; P(end,:) + 1e-10]; %#ok<AGROW>
    end

    % --- Query point --------------------------------------------------
    switch mod(k, 5)
        case 0   % exactly on the path: d must be 0
            q = P(max(1, floor(n/2)), :);
        case 1   % well before the start: s must clamp to 0
            q = P(1,:) - [20 0];
        case 2   % well past the end: s must clamp to the total length
            q = P(end,:) + [20 0];
        otherwise % general position
            q = P(max(1, floor(n/2)), :) + randn(s, 1, 2) * 5;
    end

    [sVal, dVal] = projectPointOnPath(P, q);

    % --- Write --------------------------------------------------------
    fprintf(fid, '%d', size(P,1));
    for i = 1:size(P,1)
        fprintf(fid, ',%.17g,%.17g', P(i,1), P(i,2));
    end
    fprintf(fid, ',%.17g,%.17g,%.17g,%.17g\n', q(1), q(2), sVal, dVal);
    written = written + 1;
end

nCases = written;

fprintf('Wrote %d Frenet reference cases to:\n  %s\n\n', written, outFile);
fprintf('Now run the independent Python check:\n');
fprintf('  python python/crosscheck_frenet.py "%s"\n\n', outFile);
end

% =====================================================================
function tf = isAbsolutePath(p)
%ISABSOLUTEPATH True for an absolute path on Windows or POSIX.
p = char(p);
tf = false;
if isempty(p), return; end
if p(1) == filesep || p(1) == '/'
    tf = true;
elseif numel(p) >= 2 && isletter(p(1)) && p(2) == ':'
    tf = true;
end
end
