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
    return sqrt(self.flatDistance(other)**2.0 + self.elevDiff(other)**2.0)
  def elevDiff(self, other):
    return other.alt - self.alt
  def flatDistance(self, other):
    dLat = other.lat - self.lat
    dLon = other.lon - self.lon
    a = sin(dLat/2)**2.0 + cos(self.lat) * cos(other.lat) * sin(dLon/2)**2.0
    return 2 * radius * atan2(sqrt(a), sqrt(1-a))
  def bearing(self, other):
    dLon = other.lon - self.lon
    y = sin(dLon) * cos(other.lat)
    x = cos(self.lat)*sin(other.lat) - sin(self.lat)*cos(other.lat)*cos(dLon)
    val = atan2(y, x)
    return val if val >= 0 else pi + val # bearing is in radians from north, between [0, 2Pi]

class ECEFPoint(object):
  def __init__(self, X, Y, Z):
    self.point = numpy.array([X,Y,Z])
  def distance(self, other):
    return norm(other.point - self.point)
  def elevDiff(self, other):
    return norm(other) - norm(self)
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

  def __repr__(self):
    return "<Segment: Dist=%.2f, Grade=%.2f, Elev=%.2f>" % (self.total_distance, 100*self.grade, self.elev_change)

  def timeTaken(self, power, model):
    # Apply fixed power on this segment
    # TODO: this is sorta fucked for downhill segments; we assume the terminal velocity applies to the whole segment, and if they're short it's really bad
    # TODO: similarly, for short uphills, we carry no momentum from the previous segment
    # this needs to have an initial velocity input as well, i think. we also need to model acceleration better
    grade = self.grade
    wind_alpha = self.wind_alpha
    average_vel = speedAtFixedPower(model, power, grade, wind_alpha)
    return self.total_distance / average_vel

def speedAtFixedPower(model, power, grade, alpha):
  def f(u):
    return power - powerAtFixedSpeed(model, u, grade, alpha)
  return scipy.optimize.brentq(f, 0.1, 25)

def powerAtFixedSpeed(model, speed, grade, alpha):
  hill_angle = atan(grade)
  u = speed
  v = model["wind_velocity"]
  cda = model["cda"]
  crr = model["crr"]
  rho = model["rho"]
  weight = model["weight"]
  w = sqrt( (u + v*cos(alpha))**2 + (v*sin(alpha))**2 )
  F_drag = 0.5 * cda * rho * w * (u + v*cos(alpha))
  F_rolling = 9.8067 * cos(hill_angle) * crr * weight
  F_grav = 9.8067 * sin(hill_angle) * weight
  P_hub = (F_grav + F_rolling + F_drag) * u
  return max(P_hub, 0)

def airDensity(temp, pressure, dewpoint):
  # reasonable values are shown in parens
  # temp in degrees C (25)
  # pressure in pascals (101800)
  # dew point in degrees C (7.5)
  # output is air density in kg / m^3
  c_0 = 6.1078
  c_1 = 7.5
  c_2 = 237.3
  P_v = c_0 * (10 ** ((c_1 * dewpoint)/(c_2 + dewpoint)) )
  P_d = pressure - P_v
  T_k = temp + 273.15
  R_v = 461.4964
  R_d = 287.0531

  rho = (P_d / (R_d * T_k)) + (P_v / (R_v * T_k))
  return rho

def criticalPower(time, model):
  constraints = model["power"]
  if time in constraints:
    return constraints[time]
  keys = sorted(list(constraints))
  if time < keys[0]:
    return constraints[keys[0]]
  if time > keys[-1]:
    return constraints[keys[-1]]
  max_index = 0
  while keys[max_index] < time:
    max_index += 1
  lower_bound_time = keys[max_index-1]
  upper_bound_time = keys[max_index]
  alpha = 1.0*(time - lower_bound_time) / (upper_bound_time - lower_bound_time)
  return constraints[lower_bound_time] + alpha*(constraints[upper_bound_time] - constraints[lower_bound_time])

def totalTimeAtConstantPower(segment_list, power, model):
  return sum(seg.timeTaken(power, model) for seg in segment_list)

def chooseOptimalConstantPower(segment_list, model, initial_power_guess):
  min_time = 999999999
  best_power = 0
  p_0 = initial_power_guess
  t_0 = totalTimeAtConstantPower(segment_list, p_0, model)
  improvement = min_time - t_0
  t_old = t_0
  while improvement > 0.1:
    p_new = criticalPower(t_old, model)
    t_new = totalTimeAtConstantPower(segment_list, p_new, model)
    if t_new < min_time:
      min_time = t_new
      best_power = p_new
    improvement = t_old - t_new
    t_old = t_new
  return (best_power, min_time)

def ramer_douglas_peucker(points, threshold):
  return [0] + ramer_douglas_peucker_aux(0, len(points)-1, points, threshold) + [len(points)-1]

def ramer_douglas_peucker_aux(first_ind, last_ind, points, epsilon):
  # return a list of indices ind such that points[ind[whatever]] is a control point between first_ind and last_ind, exclusive 
  if last_ind - first_ind < 2:
    return []
  firstPoint = points[first_ind]
  lastPoint = points[last_ind]
  index = first_ind-1
  dist = 0
  for i in range(first_ind+1, last_ind):
    cDist = perpendicularDist(points[i], firstPoint, lastPoint)
    if cDist > dist:
      dist = cDist
      index = i
  if dist > epsilon:
    R1 = ramer_douglas_peucker_aux(first_ind, index, points, epsilon)
    R2 = ramer_douglas_peucker_aux(index, last_ind, points, epsilon)
    return R1 + [index] + R2
  else:
    return []

def buildSegments(compressed_indices, course_data, model):
  wind_speed = model["wind_velocity"]
  wind_dir = model["wind_direction"]
  segs = []
  # Consider adjacent pairs of compressed indices
  for j in range(0, len(compressed_indices)-1):
    first_ind = compressed_indices[j]
    second_ind = compressed_indices[j+1]

    p1 = course_data[first_ind]
    p2 = course_data[second_ind]
    # TODO: decide if we want to use flat distance here instead
    aggregate_distance = 0
    for i in range(first_ind, second_ind):
        aggregate_distance += course_data[i].distance(course_data[i+1])
    seg = CourseSeg(p1, p2, aggregate_distance, wind_speed, wind_dir)
    segs.append(seg)
  return segs

def perpendicularDist(p, p1, p2):
  lineseg =   p2.point - p1.point
  resultant = p.point - p1.point
  return norm(numpy.cross(lineseg, resultant)) / norm(lineseg)

def lla2ecef(llaPoint):
  # Uses WGS84 ECEF formula
  e = 0.08181919084262149
  (lat, lon, h) = (llaPoint.lat, llaPoint.lon, llaPoint.alt)
  N = radius / sqrt(1 - (e * sin(lat))**2.0)
  X = (N + h)*cos(lat)*cos(lon)
  Y = (N + h)*cos(lat)*sin(lon)
  Z = ((0.9933056200098478)*N + h) * sin(lat)
  return ECEFPoint(X, Y, Z)

