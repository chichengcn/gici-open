function pos = xyz2plh(varargin)
%XYZ2PLH converts coordinates given in a cartesian system into 
%   lat, lon and hgt.
%
%   XYZ2PLH(xyz) converts the vector xyz containing the (x,y,z) 
%      coordinates (in order) using the WGS84 ellipsoid parameters.
%
%   XYZ2PLH(x,y,z) converts the (x,y,z) coordinates using the WGS84
%      ellipsoid parameters.
%
%   XYZ2PLH(x,y,z,a,b) converts the (x,y,z) coordinates using the 
%      ellipsoid parameters defined by a and b.
%
%   NOTE: In all cases, the return value is a 3x1 vector containing
%         (lat,lon,hgt), with lat and lon being given in units of radians.
%

switch nargin
   case 1,
      xyz = varargin{1};
      x = xyz(1);
      y = xyz(2);
      z = xyz(3);
    	a = 6378137.0;
    	b = 6356752.31425;

   case 3,
      x = varargin{1};
      y = varargin{2};
      z = varargin{3};
    	a = 6378137.0;
    	b = 6356752.31425;
      
   case 5,
      x = varargin{1};
      y = varargin{2};
      z = varargin{3};
    	a = varargin{4};
    	b = varargin{5};

end

e2 = (a^2-b^2)/(a^2);
p = sqrt(x^2+y^2);

% check for sigularity
if p<=1.0e-6
	if z>0
		pos(1,1) = pi/2.0;
	else
		pos(1,1) = -pi/2.0;
	end
	pos(2,1) = 0.0;		% longitude does not exist
	pos(3,1) = abs(z)-b;

else
	N0 = 0;
	h0 = 0;
	
	phi = atan(z/p/(1-e2));

	N1 = calcN(phi,a,b);
	h1 = p/cos(phi) - N1;
	phi = atan((z/p)/(1-e2*N1/(N1+h1)));

	while abs(N1-N0)>=0.01 & abs(h1-h0)>=0.01
		N0 = N1;
		h0 = h1;

		N1 = calcN(phi,a,b);
		h1 = p/cos(phi) - N1;
		phi = atan((z/p)/(1-e2*N1/(N1+h1)));
	end

	pos(1,1) = phi;
	pos(2,1) = atan2(y,x);		% longitude is given by a closed formula
	pos(3,1) = h1;
end		

