# Taken from utexas-art-ros-pkg:art_vehicle/applanix */

##
## Conversions between coordinate systems.
##
## Includes LatLong<->UTM.
##


## Universal Transverse Mercator transforms.
##
##     Functions to convert (spherical) latitude and longitude to and
##     from (Euclidean) UTM coordinates.
##
##     author Chuck Gantz- chuck.gantz@globalstar.com
##

import math

RADIANS_PER_DEGREE = math.pi/180.0
DEGREES_PER_RADIAN = 180.0/math.pi

# WGS84 Parameters
WGS84_A = 6378137.0		# major axis
WGS84_B = 6356752.31424518	# minor axis
WGS84_F = 0.0033528107	        # ellipsoid flattening
WGS84_E = 0.0818191908		# first eccentricity
WGS84_EP = 0.0820944379		# second eccentricity

# UTM Parameters
UTM_K0 = 0.9996			# scale factor
UTM_FE = 500000.0		# false easting
UTM_FN_N = 0.0			# false northing on north hemisphere
UTM_FN_S = 10000000.0		# false northing on south hemisphere
UTM_E2 = (WGS84_E*WGS84_E)	# e^2
UTM_E4 = (UTM_E2*UTM_E2)	# e^4
UTM_E6 = (UTM_E4*UTM_E2)	# e^6
UTM_EP2 = (UTM_E2/(1-UTM_E2))	# e'^2

def UTM(lat, lon):
    """ Utility function to convert geodetic to UTM position

    Units in are floating point degrees (sign for east/west)
    Units out are meters
    """
    # constants
    m0 = (1 - UTM_E2/4 - 3*UTM_E4/64 - 5*UTM_E6/256)
    m1 = -(3*UTM_E2/8 + 3*UTM_E4/32 + 45*UTM_E6/1024)
    m2 = (15*UTM_E4/256 + 45*UTM_E6/1024)
    m3 = -(35*UTM_E6/3072)

    # compute the central meridian
    cm = (int(lon) - int(lon)%6 + 3) if (lon >= 0.0) else (int(lon) - int(lon)%6 - 3)

    # convert degrees into radians
    rlat = lat * RADIANS_PER_DEGREE
    rlon = lon * RADIANS_PER_DEGREE
    rlon0 = cm * RADIANS_PER_DEGREE

    # compute trigonometric functions
    slat = math.sin(rlat)
    clat = math.cos(rlat)
    tlat = math.tan(rlat)

    # decide the false northing at origin
    fn = UTM_FN_N if (lat > 0) else UTM_FN_S

    T = tlat * tlat
    C = UTM_EP2 * clat * clat
    A = (rlon - rlon0) * clat
    M = WGS84_A * (m0*rlat + m1*math.sin(2*rlat)
	+ m2*math.sin(4*rlat) + m3*math.sin(6*rlat))
    V = WGS84_A / sqrt(1 - UTM_E2*slat*slat)

    # compute the easting-northing coordinates
    x = UTM_FE + UTM_K0 * V * (A + (1-T+C)*math.pow(A,3)/6
	+ (5-18*T+T*T+72*C-58*UTM_EP2)*math.pow(A,5)/120)
    y = fn + UTM_K0 * (M + V * tlat * (A*A/2
	+ (5-T+9*C+4*C*C)*math.pow(A,4)/24
	+ ((61-58*T+T*T+600*C-330*UTM_EP2)
	    * math.pow(A,6)/720)))

    return x,y


def UTMLetterDesignator(Lat):
    """ Determine the correct UTM letter designator for the
        given latitude
 
        @returns 'Z' if latitude is outside the UTM limits of 84N to 80S

        Written by Chuck Gantz- chuck.gantz@globalstar.com
    """

    # 'Z' is an error flag, the Latitude is outside the UTM limits
    LetterDesignator = 'Z'

    if ((84 >= Lat) and (Lat >= 72)):
        LetterDesignator = 'X'
    elif ((72 > Lat) and (Lat >= 64)):  
        LetterDesignator = 'W'
    elif ((64 > Lat) and (Lat >= 56)):  
        LetterDesignator = 'V'
    elif ((56 > Lat) and (Lat >= 48)):  
        LetterDesignator = 'U'
    elif ((48 > Lat) and (Lat >= 40)):  
        LetterDesignator = 'T'
    elif ((40 > Lat) and (Lat >= 32)):
        LetterDesignator = 'S'
    elif ((32 > Lat) and (Lat >= 24)):
        LetterDesignator = 'R'
    elif ((24 > Lat) and (Lat >= 16)):
        LetterDesignator = 'Q'
    elif ((16 > Lat) and (Lat >= 8)):
        LetterDesignator = 'P'
    elif (( 8 > Lat) and (Lat >= 0)):
        LetterDesignator = 'N'
    elif (( 0 > Lat) and (Lat >= -8)):
        LetterDesignator = 'M'
    elif ((-8 > Lat) and (Lat >= -16)):
        LetterDesignator = 'L'
    elif((-16 > Lat) and (Lat >= -24)):
        LetterDesignator = 'K'
    elif((-24 > Lat) and (Lat >= -32)):
        LetterDesignator = 'J'
    elif((-32 > Lat) and (Lat >= -40)): 
        LetterDesignator = 'H'
    elif((-40 > Lat) and (Lat >= -48)): 
        LetterDesignator = 'G'
    elif((-48 > Lat) and (Lat >= -56)):
        LetterDesignator = 'F'
    elif((-56 > Lat) and (Lat >= -64)): 
        LetterDesignator = 'E'
    elif((-64 > Lat) and (Lat >= -72)): 
        LetterDesignator = 'D'
    elif((-72 > Lat) and (Lat >= -80)): 
        LetterDesignator = 'C'

    return LetterDesignator


def LLtoUTM(Lat, Long):
    """
    Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 
    East Longitudes are positive, West longitudes are negative.
    North latitudes are positive, South latitudes are negative
    Lat and Long are in fractional degrees
 
    Written by Chuck Gantz- chuck.gantz@globalstar.com
    """
    a = WGS84_A
    eccSquared = UTM_E2
    k0 = UTM_K0

    # Make sure the longitude is between -180.00 .. 179.9
    LongTemp = (Long+180)-int((Long+180)/360)*360-180

    LatRad = Lat*RADIANS_PER_DEGREE
    LongRad = LongTemp*RADIANS_PER_DEGREE

    ZoneNumber = int((LongTemp + 180)/6) + 1

    if( Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0 ):
        ZoneNumber = 32

    # Special zones for Svalbard
    if( Lat >= 72.0 and Lat < 84.0 ):
	
        if( LongTemp >= 0.0 and LongTemp <  9.0 ): 
            ZoneNumber = 31
        elif( LongTemp >= 9.0 and LongTemp < 21.0 ): 
            ZoneNumber = 33
	elif( LongTemp >= 21.0 and LongTemp < 33.0 ): 
	    ZoneNumber = 35
	elif( LongTemp >= 33.0 and LongTemp < 42.0 ): 
	    ZoneNumber = 37
	
    # +3 puts origin in middle of zone
    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3
    LongOriginRad = LongOrigin * RADIANS_PER_DEGREE

    # compute the UTM Zone from the latitude and longitude
    UTMZone = "%d%c" % (ZoneNumber, UTMLetterDesignator(Lat))

    eccPrimeSquared = (eccSquared)/(1-eccSquared)

    N = a/math.sqrt(1-eccSquared*math.sin(LatRad)*math.sin(LatRad))
    T = math.tan(LatRad)*math.tan(LatRad)
    C = eccPrimeSquared*math.cos(LatRad)*math.cos(LatRad)
    A = math.cos(LatRad)*(LongRad-LongOriginRad)

    M = a*((1	- eccSquared/4		- 3*eccSquared*eccSquared/64	- 5*eccSquared*eccSquared*eccSquared/256)*LatRad - \
	    (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024)*math.sin(2*LatRad) + \
	    (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*math.sin(4*LatRad) - \
	    (35*eccSquared*eccSquared*eccSquared/3072)*math.sin(6*LatRad))

    UTMEasting = float(k0*N*(A+(1-T+C)*A*A*A/6 +\
		    (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120) + \
		    500000.0)

    UTMNorthing = float(k0*(M+N*math.tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24 + \
		    (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)))
    if(Lat < 0):
        UTMNorthing += 10000000.0  # 10000000 meter offset for southern hemisphere

    return UTMNorthing, UTMEasting, UTMZone 


def UTMtoLL(UTMNorthing, UTMEasting, UTMZone):
    """
    Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
 
    East Longitudes are positive, West longitudes are negative.
    North latitudes are positive, South latitudes are negative
    Lat and Long are in fractional degrees.
 
    Written by Chuck Gantz- chuck.gantz@globalstar.com
    """
	
    k0 = UTM_K0
    a = WGS84_A
    eccSquared = UTM_E2
    e1 = (1-sqrt(1-eccSquared))/(1+sqrt(1-eccSquared))

    x = UTMEasting - 500000.0   # remove 500,000 meter offset for longitude
    y = UTMNorthing

    ZoneNumber = int(UTMZone[0:-1])
    ZoneLetter = str(UTMZone[-1])
    if((ZoneLetter - 'N') >= 0):
	NorthernHemisphere = 1  # point is in northern hemisphere
    else:
	NorthernHemisphere = 0  # point is in southern hemisphere
	y -= 10000000.0         # remove 10,000,000 meter offset used for southern hemisphere
	

    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3       # +3 puts origin in middle of zone

    eccPrimeSquared = (eccSquared)/(1-eccSquared)

    M = y / k0
    mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256))

    phi1Rad = mu + (3*e1/2-27*e1*e1*e1/32)*math.sin(2*mu) + \
            (21*e1*e1/16-55*e1*e1*e1*e1/32)*math.sin(4*mu) + \
                    (151*e1*e1*e1/96)*math.sin(6*mu)
    phi1 = phi1Rad*DEGREES_PER_RADIAN

    N1 = a/math.sqrt(1-eccSquared*math.sin(phi1Rad)*math.sin(phi1Rad))
    T1 = math.tan(phi1Rad)*math.tan(phi1Rad)
    C1 = eccPrimeSquared*math.cos(phi1Rad)*math.cos(phi1Rad)
    R1 = a*(1-eccSquared)/math.pow(1-eccSquared*math.sin(phi1Rad)*math.sin(phi1Rad), 1.5)
    D = x/(N1*k0)

    Lat = phi1Rad - (N1*math.tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24 + \
		(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720)
    Lat = Lat * DEGREES_PER_RADIAN

    Long = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1) * \
			D*D*D*D*D/120)/math.cos(phi1Rad)
    Long = LongOrigin + Long * DEGREES_PER_RADIAN

    return Lat, Long
