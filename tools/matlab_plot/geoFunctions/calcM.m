function M = calcM(lat,a,b)
% CALCM calculates the meridian radius of curvature at a given
%   latitude for a given ellipsoid.
%
%   CALCM(lat) calculates the meridian radius of curvature using
%      the WGS84 ellipsoid parameters.
%   CLACM(lat,a,b) calculates the meridian radius of curvature using
%      the ellipsoid parameters specified by 'a' and 'b'.
%
%   The latitude parameter should be given in radians.
%
if nargin<=2
	a = 6378137.0;
	b = 6356752.31425;
end

e2 = (a^2-b^2)/a^2;
W  = sqrt(1.0-e2*((sin(lat)).^2));
M  = (a*(1.0-e2))./W.^3;