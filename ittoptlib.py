from numpy.linalg import norm
import numpy
import scipy.optimize
from math import sin, cos, atan2, sqrt, radians, degrees, pi, atan

radius = 6378137

class LLAPoint(object):
  def __init__(self, lat, lon, alt):
    # Assume lat, lon are in radians. Alt is in meters.
    self.lat = lat
    self.lon = lon
    self.alt = alt
  def distance(self, other):
    return sqrt(self.flatDistance(other)**2.0 + self.elevDist(other)**2.0)
  def elevDiff(self, other):
    return other.alt - self.alt
  def flatDistance(self, other):
    dLat = other.lat - self.lat
    dLon = other.lon - self.lon
    a = sin(dLat/2)**2.0 + cos(la1) * cos(la2) * sin(dLon/2)**2.0
    return 2 * radius * atan2(sqrt(a), sqrt(1-a))
  def bearing(self, other):
    dLon = other.lon - self.lon
    y = sin(dLon) * cos(other.lat)
    x = cos(self.lat)*sin(other.lat) - sin(self.lat)*cos(other.lat)*cos(dLon)
    val = atan2(y, x)
    return val if val >= 0 else pi + val # bearing is in radians from north, between [0, 2Pi]

class ECEFPoint(object):
  def __init__(self, X, Y, Z):
    self.X = X
    self.Y = Y
    self.Z = Z
  def distance(self, other):
    a = numpy.array((self.X, self.Y, self.Z))
    b = numpy.array((other.X, other.Y, other.Z))
    return norm(b - a)
  def elevDiff(self, other):
    a = numpy.array((self.X, self.Y, self.Z))
    b = numpy.array((other.X, other.Y, other.Z))
    return norm(b) - norm(a)
  def flatDistance(self, other):
    return sqrt(self.distance(other)**2.0 - self.elevDiff(other)**2.0)

class CourseSeg(object):
  def __init__(self, lla_1, lla_2, total_distance, wind_speed, wind_dir):
    # When you create the segment, make sure to total up all the intermediate bits
    self.lla_1 = lla_1
    self.lla_2 = lla_2
    self.ecef_1 = lla2ecef(lla_1)
    self.ecef_2 = lla2ecef(lla_2)
    self.total_distance = total_distance
    self.elev_change = lla_1.elevDiff(lla_2)
    self.grade = self.elev_change / self.total_distance
    self.wind_speed = wind_speed
    self.bearing = lla_1.bearing(lla_2)
    self.wind_alpha = pi - ((self.bearing - wind_dir) % (2*pi)) 
    # the relative angle of the wind, zero is "head on", +/- pi is "tail wind"

  def lla_str(self):
    return "<Segment (lla): dist=%.2f, elev_change=%.2f, grade=%.2f, bearing=%.4f>" % (self.total_distance_lla, self.elev_change_lla, 100*self.grade_lla, degrees(self.bearing))

  def ecef_str(self):
    return "<Segment (ecef): dist=%.2f, elev_change=%.2f, grade=%.2f, bearing=%.4f>" % (self.total_distance_ecef, self.elev_change_ecef, 100*self.grade_ecef, degrees(self.bearing))

  def timeTaken(self, power, weight, crr, cda, rho):
    # Apply fixed power on this segment
    # TODO: this is sorta fucked for downhill segments; we assume the terminal velocity applies to the whole segment, and if they're short it's really bad
    # TODO: similarly, for short uphills, we carry no momentum from the previous segment
    # this needs to have an initial velocity input as well, i think. we also need to model acceleration better
    average_vel = speedAtFixedPower(weight, self.grade, crr, cda, rho, power, self.wind_speed, self.wind_alpha)
    print "average vel %f" % average_vel
    return self.total_distance_lla / average_vel



def lla2ecef(llaPoint):
  # Uses WGS84 ECEF formula
  e = 0.08181919084262149
  (lat, lon, h) = (llaPoint.lat, llaPoint.lon, llaPoint.alt)
  N = radius / sqrt(1 - (e * sin(lat))**2.0)
  X = (N + h)*cos(lat)*cos(lon)
  Y = (N + h)*cos(lat)*sin(lon)
  Z = ((0.9933056200098478)*N + h) * sin(lat)
  return ECEFPoint(X, Y, Z)

