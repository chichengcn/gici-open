function N = calcN(lat,a,b)
% CALCN calculates the prime vertical radius of curvature at a given
%   latitude for a given ellipsoid.
%
%   CALCN(lat) calculates the prime vertical radius of curvature using
%      the WGS84 ellipsoid parameters.
%   CLACN(lat,a,b) calculates the prime vertical radius of curvature 
%      using the ellipsoid parameters specified by 'a' and 'b'.
%
%   The latitude parameter should be given in radians.
%

if nargin<=2
	a = 6378137.0;
	b = 6356752.31425;
end

e2 = (a^2-b^2)/(a^2);
W  = sqrt(1.0-e2*((sin(lat)).^2));
N  = a./W;