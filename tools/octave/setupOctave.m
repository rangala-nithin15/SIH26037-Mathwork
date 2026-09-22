function setupOctave()
%SETUPOCTAVE Prepare GNU Octave to run this project (compatibility shims).
%
%   OCTAVE ONLY. Runs setupPaths and then adds tools/octave (RandStream and
%   matlab.unittest verification shims). Never call this from MATLAB: the
%   shims would shadow MATLAB's own RandStream and functiontests.
%
%   Results obtained under Octave must be reported as Octave results. Random
%   numbers differ from MATLAB's (see RandStream.m in this folder).

if exist('OCTAVE_VERSION', 'builtin') == 0
    error('setupOctave:notOctave', 'setupOctave is for GNU Octave only. In MATLAB run setupPaths.');
end
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
run(fullfile(root, 'setupPaths.m'));
addpath(here);
warning('off', 'Octave:shadowed-function');
fprintf('Octave %s: compatibility shims added from %s\n', OCTAVE_VERSION, here);
end
