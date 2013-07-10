import argparse
import glob
import os
from numpy.linalg import norm
import numpy
import scipy.optimize
from math import sin, cos, atan2, sqrt, radians, degrees, pi, atan

radius = 6378137 # radius of earth in meters

class CourseSeg(object):
  def __init__(self, lla_1, lla_2, wind_speed, wind_dir):
    self.lla_1 = lla_1
    self.lla_2 = lla_2
    self.ecef_1 = lla2ecef(lla_1)
    self.ecef_2 = lla2ecef(lla_2)
    (la1, lo1, al1) = lla_1
    (la2, lo2, al2) = lla_2
    la1 = radians(la1)
    lo1 = radians(lo1)
    la2 = radians(la2)
    lo2 = radians(lo2)
    dLat = la2 - la1
    dLon = lo2 - lo1
    a = sin(dLat/2)**2.0 + cos(la1) * cos(la2) * sin(dLon/2)**2.0
    self.flat_distance_lla = radius * 2 * atan2(sqrt(a), sqrt(1-a))
    self.elev_change_lla = al2 - al1
    self.elev_change_ecef = norm(self.ecef_2) - norm(self.ecef_1)
    self.total_distance_lla = sqrt(self.elev_change_lla**2.0 + self.flat_distance_lla**2.0)
    self.total_distance_ecef = norm(self.ecef_2 - self.ecef_1)
    self.flat_distance_ecef = sqrt(self.total_distance_ecef**2.0 - self.elev_change_ecef**2.0)
    self.grade_lla = self.elev_change_lla / self.flat_distance_lla
    self.grade_ecef = self.elev_change_ecef / self.flat_distance_ecef
    y = sin(dLon) * cos(la2)
    x = cos(la1)*sin(la2) - sin(la1)*cos(la2)*cos(dLon)
    val = atan2(y, x)
    self.bearing = val if val >= 0 else pi + val # bearing is in radians from north, between [0, 2Pi]
    self.wind_speed = wind_speed 
    self.wind_alpha = pi - ((self.bearing - wind_dir) % (2*pi)) # the relative angle of the wind, zero is "head on", +/- pi is "tail wind"

  def lla_str(self):
    return "<Segment (lla): dist=%.2f, elev_change=%.2f, grade=%.2f, bearing=%.4f>" % (self.total_distance_lla, self.elev_change_lla, 100*self.grade_lla, degrees(self.bearing))
  
  def ecef_str(self):
    return "<Segment (ecef): dist=%.2f, elev_change=%.2f, grade=%.2f, bearing=%.4f>" % (self.total_distance_ecef, self.elev_change_ecef, 100*self.grade_ecef, degrees(self.bearing))

  def timeTaken(self, power, weight, crr, cda, rho):
    # Apply fixed power on this segment
    # TODO: this is sorta fucked for downhill segments; we assume the terminal velocity applies to the whole segment, and if they're short it's really bad
    # TODO: similarly, for short uphills, we carry no momentum from the previous segment
    # this needs to have an initial velocity input as well, i think. we also need to model acceleration better
    average_vel = speedAtFixedPower(weight, self.grade_lla, crr, cda, rho, power, self.wind_speed, self.wind_alpha)
    print "average vel %f" % average_vel
    return self.total_distance_lla / average_vel


def lla2ecef(p):
  # Uses WGS84 ECEF formula
  a = 6378137.0
  e = 0.08181919084262149
  (lat, lon, h) = p
  lat_radians = radians(lat)
  lon_radians = radians(lon)
  N = a / sqrt(1 - (e * sin(lat_radians))**2.0)
  X = (N + h)*cos(lat_radians)*cos(lon_radians)
  Y = (N + h)*cos(lat_radians)*sin(lon_radians)
  Z = ((0.9933056200098478)*N + h) * sin(lat_radians)
  return numpy.array((X, Y, Z))

def main(args):
  base = args.base
  verbose = args.verbose
  compress = args.compress
  
  if verbose:
    print "Launching optimization in verbose mode..."
  print "Reading directory %s" % base
  param_files = glob.glob(os.path.join(base, '*.par'))
  if not param_files:
    print "Could not find a file matching *.par expansion."
    return
  if len(param_files) > 1:
    print "Found too many parameter files in the base directory."
    return
  param_file = param_files[0]
  if verbose:
    print "Using parameter file %s" % param_file
  (rider_info, environment_info, course_file) = parseParameterFile(param_file)
  if verbose:
    print "Loaded rider info: %s" % str(rider_info)
    print "Loaded environment info: %s" % str(environment_info)
    print "Loading course file %s" % course_file

  full_course_file = os.path.join(base, course_file)
  if not os.path.exists(full_course_file):
    print "Could not find course file %s" % full_course_file
    return
  lat_lon_alt = parseCourseFile(full_course_file)
  if compress:
    print compress
    ecef_points = map(lla2ecef, lat_lon_alt)
    compression_threshold = 35
    compressed_indices = ramer_douglas_peucker(ecef_points, compression_threshold)
    compressed_lat_lon_alt = [lat_lon_alt[i] for i in compressed_indices]
    segment_list = buildSegments(compressed_lat_lon_alt, environment_info['WIND_VELOCITY'], environment_info['WIND_DIRECTION'])
  else:
    segment_list = buildSegments(lat_lon_alt, environment_info['WIND_VELOCITY'], environment_info['WIND_DIRECTION'])

  total_elev = 0
  total_dist = 0
  for segment in segment_list:
    print segment.ecef_str()
    print segment.lla_str()
    print "--------------------"
    total_elev += segment.elev_change_lla
    total_dist += segment.total_distance_lla
  
  if verbose:
    if compress:
      print "Original course had %d points, new course has %d points." % (len(ecef_points), len(compressed_lat_lon_alt))
    else:
      print "Course has %d points." % len(lat_lon_alt)
    print "Total elev change %.2f, total dist %.2f" % (total_elev, total_dist)
  constraints = buildCriticalPower(rider_info)
  weight = rider_info["RIDER_WEIGHT"] + rider_info["BIKE_WEIGHT"]
  crr = rider_info["RIDER_CRR"]
  cda = rider_info["RIDER_CDA"]
  air_density = airDensity(environment_info['AIR_TEMP'], environment_info['AIR_PRESSURE'], environment_info['DEW_POINT'])
  initial_power_guess = 240
  chooseOptimalConstantPower(segment_list, constraints, weight, crr, cda, air_density, initial_power_guess)
    
def parseParameterFile(param_file):
  rider_info = {}
  environment_info = {}
  course_file = ""
  with open(param_file) as f:
    # Sections are separated by white space, so we look for the blank lines.
    lines = f.read()
    rider, env, course = lines.split('\n\n')
    for line in rider.splitlines():
      if not line.startswith('#'):
        key, val = line.split(",")
        rider_info[key] = float(val)
    for line in env.splitlines():
      if not line.startswith('#'):
        key, val = line.split(",")
        environment_info[key] = float(val)
    for line in course.splitlines():
      if not line.startswith('#'):
        course_file = line.split(", ")[1]
  return rider_info, environment_info, course_file

def buildCriticalPower(rider_info):
  d = {}
  d[30] = rider_info["POWER_30"]
  d[60] = rider_info["POWER_60"]
  d[300] = rider_info["POWER_300"]
  d[600] = rider_info["POWER_600"]
  d[120] = rider_info["POWER_1200"]
  d[1800] = rider_info["POWER_1800"]
  d[3600] = rider_info["POWER_3600"]
  return d
  
def parseCourseFile(course_file):
  lat_lon_alt = []
  with open(course_file) as f:
    lines = f.readlines()
    started = False
    finished = False
    for line in lines:
      if "<coordinates>" in line:
        started = True
        continue
      if "</coordinates>" in line:
        finished = True
      if started and not finished:
        lon, lat, alt = map(float, line.split(","))
        lat_lon_alt.append((lat, lon, alt))
  return lat_lon_alt

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

def perpendicularDist(p, p1, p2):
  lineseg = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
  resultant = (p[0] - p1[0], p[1] - p1[1], p[2] - p1[2])
  return norm(numpy.cross(lineseg, resultant)) / norm(lineseg)

def buildSegments(lat_lon_alt, wind_speed, wind_dir):
  # For each adjacent lat_lon_alt point, constructs a segment between them 
  segs = []
  for i in range(0, len(lat_lon_alt)-1):
    p1 = lat_lon_alt[i]
    p2 = lat_lon_alt[i+1]
    seg = CourseSeg(p1, p2, wind_speed, wind_dir)
    segs.append(seg)
  return segs

def powerAtFixedSpeed(weight, grade, crr, cda, rho, u, v, a):
  # weight is total weight (rider+bike) in kg
  # grade is a float (0.0-1.0) (unitless)
  # crr is a float (unitless)
  # cda is a float (m^2)
  # rho is a float (kg / m^3) for air density
  # u is "bike velocity" a float (m/s)
  # v is "wind velocity" a float (m/s)
  # a is  RELATIVE wind angle to rider's direction (zero = head on, +/- pi = tail on)
  G = 9.8067
  hill_angle = atan(grade)
  #F_drag = 0.5 * cda * rho * u * u
  w = sqrt( (u + v*cos(a))**2 + (v*sin(a))**2 )
  F_drag = 0.5 * cda * rho * w * (u + v*cos(a))
  F_rolling = G * cos(hill_angle) * crr * weight
  F_grav = G * sin(hill_angle) * weight
  P_hub = (F_grav + F_rolling + F_drag) * u
  # it's possible we have to do "negative power" to counteract gravity - that is, gravity alone will take us to a a speed above $u$ m/s
  return max(P_hub, 0)
 

def speedAtFixedPower(weight, grade, crr, cda, rho, power, v, a):
  # the "inverse" of powerAtFixedSpeed.
  def f(u):
    # returns difference in power between actual, targetted power, and power required to sustain a speed
    # output is in watts
    return power - powerAtFixedSpeed(weight, grade, crr, cda, rho, u, v, a)

  return scipy.optimize.brentq(f, 0.1, 25)


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

def criticalPower(time, constraints):
  if time in constraints:
    return constraints[time]

  # Does linear interpolation between relevant discrete constraints
  # constraints is a dictionary mapping timeintervalsec:power
  keys = sorted(list(constraints))
  if time < keys[0]:
    # critical power for something before the "thirty second" power is just thirty second power
    print "requested time before CP constraints start"
    return constraints[keys[0]]
  if time > keys[-1]:
    print "requested time after CP constraints end"
    # critical power for something beyond "hour power" is just hour power
    return constraints[keys[-1]]
  # identify lower and upper bound, and linear interpolate
  max_index = 0
  while keys[max_index] < time:
    max_index += 1

  lower_bound_time = keys[max_index-1]
  upper_bound_time = keys[max_index]
  alpha = 1.0*(time - lower_bound_time) / (upper_bound_time - lower_bound_time)
  return constraints[lower_bound_time] + alpha*(constraints[upper_bound_time] - constraints[lower_bound_time])

def cumulativeTimeTakenAtPower(segment_list, power, weight, crr, cda, rho):
  time = 0
  for segment in segment_list:
    time += segment.timeTaken(power, weight, crr, cda, rho)
  return time

def chooseOptimalConstantPower(segment_list, constraints, weight, crr, cda, rho, initial_power_guess):
  min_time = 99999999
  best_power = 0
  p_0 = initial_power_guess
  t_0 = cumulativeTimeTakenAtPower(segment_list, p_0, weight, crr, cda, rho)
  
  print "Initial power guess: %f. Time taken at this power: %f" % (p_0, t_0)

  improvement = min_time - t_0
  t_old = t_0
  while improvement > 0.1:
    p_new = criticalPower(t_old, constraints)
    t_new = cumulativeTimeTakenAtPower(segment_list, p_new, weight, crr, cda, rho)
    print "New power p_new is %.2f, new time is %.2f" % (p_new, t_new)
    if t_new < min_time:
      min_time = t_new
      best_power = p_new
    improvement = t_old - t_new
    t_old = t_new
  return best_power


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("base", help="The base directory containing experiment parameters.")
  parser.add_argument("-v", "--verbose", help="Enable verbose output.", action="store_true")
  parser.add_argument("-c", "--compress", help="Enable Ramer-Douglas-Peucker polyline decimation.", action="store_true")
  args = parser.parse_args()
  main(args)

