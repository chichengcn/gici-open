function pos = plh2xyz(varargin)
%PLH2XYZ converts coordinates given in lat, lon and hgt into a
%   cartesian system.
%
%   PLH2XYZ(plh) converts the vector plh containing the (lat,lon,hgt) 
%      coordinates using the WGS84 ellipsoid parameters.
%
%   PLH2XYZ(lat,lon,hgt) converts the (lat,lon,hgt) coordinates using 
%      the WGS84 ellipsoid parameters.
%
%   PLH2XYZ(lat,lon,hgt,a,b) converts the (lat,lon,hgt)  coordinates using 
%      the ellipsoid parameters defined by a and b.
%
%   NOTE: 
%     1) In all cases, the return value is a 3x1 vector containing (x,y,z).
%     2) All the latitude (lat) and longitude (lon) should be provided in units of radians.
%

switch nargin
   case 1,
      plh = varargin{1};
      lat = plh(1);
      lon = plh(2);
      hgt = plh(3);
    	a = 6378137.0;
    	b = 6356752.31425;

   case 3,
      lat = varargin{1};
      lon = varargin{2};
      hgt = varargin{3};
    	a = 6378137.0;
    	b = 6356752.31425;
      
   case 5,
      lat = varargin{1};
      lon = varargin{2};
      hgt = varargin{3};
    	a = varargin{4};
    	b = varargin{5};

end

% Coarse check for non-radian latitude or longitude 
% (i.e., if latitude is greater than pi/2 radians or longitude is greater than 2*pi radians)
if abs(lat) > pi/2 | abs(lon) > 2*pi
   warning( 'In plh2xyz(), the input latitude and/or longitude may not be in units of radians.' );
end

e2 = (a^2-b^2)/(a^2);
N = calcN( lat, a, b );

pos(:,1) = ( N + hgt ) .* cos( lat ) .* cos( lon );
pos(:,2) = ( N + hgt ) .* cos( lat ) .* sin( lon );
pos(:,3) = ( ( 1.0 - e2 ) .* N + hgt ) .* sin( lat );
