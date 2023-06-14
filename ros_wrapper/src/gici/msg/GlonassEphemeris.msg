string prn        # Satellite PRN number
uint16 week       # GPS week
int8 frq          # Satellite frequency number
uint8 iode        # Issue of data, ephemeris
uint8 svh         # SV health
uint8 age         # Age of operation
float64 toe       # Epoch of epherides
float64 tof       # Message frame time
float64[] pos     # Satellite position
float64[] vel     # Satellite velocity
float64[] acc     # Satellite acceleration
float64 taun      # SV clock bias
float64 gamn      # SV relative freq bias
float64 dtaun     # Delay between L1 and L2