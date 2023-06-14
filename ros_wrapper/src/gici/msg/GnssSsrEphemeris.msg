string prn        # Satellite PRN number
uint16 week       # GPS week, SSR message
float64 tow       # GPS time of week, SSR message
uint32 iod        # Issue of data, SSR message
float64 udi       # SSR update interval
uint8 iode        # Issue of data, ephemeris
uint32 iodcrc     # Issue of data crc for BDS
uint8 refd        # Sat ref datum (0:ITRF,1:regional)
float64[] deph    # Delta orbit {radial,along,cross}
float64[] ddeph   # Dot delta orbit {radial,along,cross}
float64[] dclk    # Delta clock {c0,c1,c2}
