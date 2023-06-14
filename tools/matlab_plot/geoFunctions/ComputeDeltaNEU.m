function [ dN, dE, dU ] = ComputeDeltaNEU( varargin )
%
% ComputeDeltaNEU( lat, lon, hgt, [refLat, [refLon, [refHgt]]] ) computes the 
%    change in North, East and Up, relative to the reference location.  If no
%    reference location is given, the change is computed relative to the first
%    element in each of the arrays (i.e. lat(1,1), lon(1,1), and hgt(1,1)).  
%
%    The return values are ordered [ dNorth, dEast, dUp ].  Only those that are
%    "requested" via the "nargout" parameter are computed.
%
%    Latitude and longitude parameters are expected in units of radians, and height
%    in units of metres.
%

% check is output is requested
if nargout == 0
   return;
end

% check for proper number of input parameters
if nargin < 3
   error( 'At least three parameters are necessary' );
end

% check the sizes of the input arrays
if size( varargin{1} ) ~= size( varargin{2} ) | size( varargin{1} ) ~= size( varargin{3} )
   error( 'Input array sizes must match' );
end

% store the data in a more useful form
lat = varargin{1};
lon = varargin{2};
hgt = varargin{3};

% get reference position
switch nargin
case 3,
   refLat = lat(1,1);
   refLon = lon(1,1);
   refHgt = hgt(1,1);
case 4,
   refLat = varargin{4};
   refLon = lon(1,1);
   refHgt = hgt(1,1);
case 5,
   refLat = varargin{4};
   refLon = varargin{5};
   refHgt = hgt(1,1);
otherwise,
   refLat = varargin{4};
   refLon = varargin{5};
   refHgt = varargin{6};
end

% check the size of the reference positions
if ( length( refLat ) ~= 1 & size( refLat ) ~= size( lat ) ) | ...
      ( length( refLon ) ~= 1 & size( refLon ) ~= size( lon ) ) | ...
      ( length( refHgt ) ~= 1 & size( refHgt ) ~= size( hgt ) )
   error( 'Reference positions must be single values or of the same size as the input' );
end

% compute the change in North, East and Up
if nargout ~= 0
   dN = ( lat - refLat ) .* ( calcM( lat ) + hgt );
end

if nargout > 1
   dE = ( lon - refLon ) .* ( calcN( lat ) + hgt ) .* cos( lat );
end

if nargout > 2
   dU = hgt - refHgt;
end
