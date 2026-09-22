function verifyEqual(tc, varargin) %#ok<INUSL>
%VERIFYEQUAL Octave compatibility shim for matlab.unittest verification.
%   OCTAVE ONLY. Throws 'octaveShim:verifyFailed' on failure.
a = varargin{1}; b = varargin{2}; tol = 0; rel = 0; msgIdx = 3;
k = 3;
while k <= numel(varargin)
    if ischar(varargin{k}) && strcmpi(varargin{k}, 'AbsTol'), tol = varargin{k+1}; k = k + 2; msgIdx = k;
    elseif ischar(varargin{k}) && strcmpi(varargin{k}, 'RelTol'), rel = varargin{k+1}; k = k + 2; msgIdx = k;
    else, break; end
end
ok = false;
if isnumeric(a) && isnumeric(b) || islogical(a) && islogical(b) || isnumeric(a) && islogical(b) || islogical(a) && isnumeric(b)
    if isequal(size(a), size(b)) || isscalar(b)
        d = abs(double(a) - double(b));
        lim = tol + rel * abs(double(b));
        d(isnan(a) & isnan(b)) = 0;
        ok = all(d(:) <= lim(:) | (isinf(a(:)) & isinf(b(:)) & sign(a(:)) == sign(b(:))));
    end
else
    ok = isequal(a, b);
end
if ~ok, fail(varargin, msgIdx, 'verifyEqual'); end
end

function fail(args, msgIdx, what)
msg = '';
if numel(args) >= msgIdx && ischar(args{msgIdx})
    msg = args{msgIdx};
end
error('octaveShim:verifyFailed', '%s failed. %s', what, msg);
end
