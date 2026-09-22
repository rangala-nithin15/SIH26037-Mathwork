function verifyError(tc, varargin) %#ok<INUSL>
%VERIFYERROR Octave compatibility shim for matlab.unittest verification.
%   OCTAVE ONLY. Throws 'octaveShim:verifyFailed' on failure.
f = varargin{1}; id = varargin{2}; thrown = '';
try
    f();
catch err
    thrown = err.identifier;
end
if ~strcmp(thrown, id), fail(varargin, 3, sprintf('verifyError: expected %s, got "%s"', id, thrown)); end
end

function fail(args, msgIdx, what)
msg = '';
if numel(args) >= msgIdx && ischar(args{msgIdx})
    msg = args{msgIdx};
end
error('octaveShim:verifyFailed', '%s failed. %s', what, msg);
end
