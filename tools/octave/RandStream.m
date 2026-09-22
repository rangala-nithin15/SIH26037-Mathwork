classdef RandStream < handle
%RANDSTREAM Minimal seeded random stream for running this project in GNU Octave.
%
%   OCTAVE COMPATIBILITY SHIM -- NOT USED BY MATLAB.
%
%   GNU Octave has no RandStream class. This shim implements the small part
%   of its interface the project uses -- RandStream('mt19937ar','Seed',s),
%   rand(stream), randn(stream), randi(stream, n), with optional sizes -- so
%   the simulation can be executed and checked in Octave.
%
%   The generator is MRG32k3a (L'Ecuyer), implemented in exact double
%   arithmetic; normals use Box-Muller. The numbers therefore DIFFER from
%   MATLAB's Mersenne Twister: an Octave run and a MATLAB run with the same
%   seed are not identical. Reproducibility WITHIN Octave is preserved.
%
%   This folder is added to the path only by tools/octave/setupOctave.m,
%   never by setupPaths.m.

    properties (Access = private)
        s1 = [12345 12345 12345];
        s2 = [12345 12345 12345];
        spare = NaN;
    end

    methods
        function obj = RandStream(varargin)
            seed = 0;
            for i = 1:numel(varargin)-1
                if ischar(varargin{i}) && strcmpi(varargin{i}, 'Seed')
                    seed = varargin{i+1};
                end
            end
            m1 = 4294967087;  m2 = 4294944443;
            base = mod(floor(abs(seed)), 2147483647);
            obj.s1 = mod([12345 + base, 23456 + 7*base, 34567 + 13*base], m1 - 1) + 1;
            obj.s2 = mod([45678 + 3*base, 56789 + 11*base, 67890 + 17*base], m2 - 1) + 1;
            for k = 1:20
                nextU(obj);    % warm up
            end
        end

        function u = rand(obj, varargin)
            sz = sizeArgs(varargin{:});
            u = zeros(sz);
            for i = 1:numel(u)
                u(i) = nextU(obj);
            end
        end

        function z = randn(obj, varargin)
            sz = sizeArgs(varargin{:});
            z = zeros(sz);
            for i = 1:numel(z)
                if ~isnan(obj.spare)
                    z(i) = obj.spare;
                    obj.spare = NaN;
                else
                    u1 = nextU(obj);  u2 = nextU(obj);
                    r  = sqrt(-2 * log(u1));
                    z(i) = r * cos(2*pi*u2);
                    obj.spare = r * sin(2*pi*u2);
                end
            end
        end

        function k = randi(obj, imax, varargin)
            sz = sizeArgs(varargin{:});
            k = zeros(sz);
            for i = 1:numel(k)
                k(i) = min(floor(nextU(obj) * imax) + 1, imax);
            end
        end
    end

    methods (Access = private)
        function u = nextU(obj)
            m1 = 4294967087;  m2 = 4294944443;
            p1 = mod(1403580 * obj.s1(2) - 810728 * obj.s1(1), m1);
            obj.s1 = [obj.s1(2), obj.s1(3), p1];
            p2 = mod(527612 * obj.s2(3) - 1370589 * obj.s2(1), m2);
            obj.s2 = [obj.s2(2), obj.s2(3), p2];
            d = mod(p1 - p2, m1);
            if d == 0
                d = m1;
            end
            u = d / (m1 + 1);
        end
    end
end

function sz = sizeArgs(varargin)
if isempty(varargin)
    sz = [1 1];
elseif numel(varargin) == 1
    v = varargin{1};
    if isscalar(v), sz = [v v]; else, sz = v; end
else
    sz = [varargin{:}];
end
end
