import argparse
import glob
import os
import numpy
import scipy.optimize
import math

earth_rad = 6371000 # radius of earth in metersi

class CourseSeg(object):
  def __init__(self, p1, p2, wind_speed, wind_dir):
    # p1, p2 are (lat, lon, alt) tuples
    (la1, lo1, al1) = p1
    (la2, lo2, al2) = p2
    self.elev_change = al2 - al1 # elevation change in meters
    dLat = math.radians(la2 - la1)
    dLon = math.radians(lo2 - lo1)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(math.radians(la1)) * math.cos(math.radians(la2)) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = earth_rad * c
    self.flat_distance = d
    y = math.sin(dLon) * math.cos(math.radians(la2))
    x = math.cos(math.radians(la1))*math.sin(math.radians(la2)) - math.sin(math.radians(la1))*math.cos(math.radians(la2))*math.cos(dLon)
    self.bearing = math.atan2(y, x) if math.atan2(y, x) >= 0 else math.pi + math.atan2(y, x) # bearing is in radians from north, between [0, 2Pi]
    
    self.grade = 1.0*self.elev_change / self.flat_distance # average grade of this segment, as rise over run (45 degrees is 100%)
    self.total_distance = math.sqrt(self.elev_change**2.0 + self.flat_distance**2.0) 
    self.wind_speed = wind_speed
    diff = (self.bearing - wind_dir) % (2*math.pi)
    self.wind_alpha = math.pi - diff # the relative angle of the wind, zero is "head on", +/- pi is "tail wind"

  def __repr__(self):
    return "<Segment: dist=%d, elev_change=%d, grade=%.2f, bearing=%.4f, wind speed=%d, wind alpha=%.4f>" % (self.total_distance, self.elev_change, 100*self.grade, self.bearing, self.wind_speed, self.wind_alpha)

  def timeTaken(self, power, weight, crr, cda, rho):
    # Apply fixed power on this segment
    average_vel = speedAtFixedPower(weight, self.grade, crr, cda, rho, power, self.wind_speed, self.wind_alpha)
    print average_vel
    return self.total_distance / average_vel

def main(args):
  base = args.base
  verbose = args.verbose
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
  if verbose:
    print "Initial array has %d points" % len(lat_lon_alt)
  reduced_lat_lon_alt = ramer_douglas_peucker(lat_lon_alt, 0.000001)
  if verbose:
    print "Reduced array has %d points" % len(reduced_lat_lon_alt)
    #for point in reduced_lat_lon_alt:
    #  print repr(point[1])+","+repr(point[0])+","+repr(point[2])
  segment_list = buildSegments(reduced_lat_lon_alt, environment_info['WIND_VELOCITY'], environment_info['WIND_DIRECTION'])
  cum_time = 0
  cum_dist = 0
  for segment in segment_list:
    print segment
    time = segment.timeTaken(300, 88, 0.005, 0.260, 1.226)
    print "Time at 300W, 88kg: %.3f" % time
    cum_time += time
    cum_dist += segment.total_distance
    print "cumulative time: %.3f" % cum_time
  print "Went %.2f meters at average speed of %.3f." % (cum_dist, cum_dist / cum_time)
    
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

def ramer_douglas_peucker(points, epsilon):
  firstPoint = points[0]
  lastPoint = points[-1]
  if len(points) < 3:
    return points
  index = -1
  dist = 0
  for i in range(1, len(points)-1):
    cDist = perpendicularDist(points[i], firstPoint, lastPoint)
    if cDist > dist:
      dist = cDist
      index = i
  if dist > epsilon:
    L1 = points[0:index+1]
    L2 = points[index:]
    R1 = ramer_douglas_peucker(L1, epsilon)
    R2 = ramer_douglas_peucker(L2, epsilon)
    return R1[0:-1]+R2
  else:
    return [firstPoint, lastPoint]

def perpendicularDist(p, p1, p2):
  #@TODO: this is unlikely to actually work the way we want. 
  #we need a better of way of figuring out the perpendicular distance 
  #(right now, it treats long/lat as if they were in units of METERS! gasp!)
  lineseg = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
  resultant = (p[0] - p1[0], p[1] - p1[1], p[2] - p1[2])
  return numpy.linalg.norm(numpy.cross(lineseg, resultant)) / numpy.linalg.norm(lineseg)

def buildSegments(lat_lon_alt, wind_speed, wind_dir):
  # For each adjacent lat_lon_alt point, constructs a segment between them 
  segs = []
  for i in range(0, len(lat_lon_alt)-1):
    p1 = lat_lon_alt[i]
    p2 = lat_lon_alt[i+1]
    seg = CourseSeg(p1, p2, wind_speed, wind_dir)
    segs.append(seg)
  return segs

def requiredPower(weight, grade, crr, cda, rho, u, v, a):
  # weight is total weight (rider+bike) in kg
  # grade is a float (0.0-1.0) (unitless)
  # crr is a float (unitless)
  # cda is a float (m^2)
  # rho is a float (kg / m^3) for air density
  # u is "bike velocity" a float (m/s)
  # v is "wind velocity" a float (m/s)
  # a is  RELATIVE wind angle to rider's direction (zero = head on, +/- pi = tail on)
  G = 9.8067
  hill_angle = math.atan(grade)
  #F_drag = 0.5 * cda * rho * u * u
  w = math.sqrt( (u + v*math.cos(a))**2 + (v*math.sin(a))**2 )
  cos_b = (u + v*math.cos(a)) / w
  F_drag = 0.5 * cda * rho * w**2.0 * cos_b
  F_rolling = G * math.cos(hill_angle) * crr * weight
  F_grav = G * math.sin(hill_angle) * weight
  P_hub = (F_grav + F_rolling + F_drag) * u
  return P_hub

def speedAtFixedPower(weight, grade, crr, cda, rho, power, v, a):
  # the "inverse" of requiredPower.
  G = 9.8067
  def f(u):
    upvca = u + v*math.cos(a)
    return ((G*u*weight)/math.sqrt(1+grade**2.0))*(crr+grade)   +   0.5*cda*rho*u * upvca * math.sqrt(upvca**2.0 + (v*math.sin(a))**2.0) - power
  def f_deriv(u):
    cosa = math.cos(a)
    upvca = u + v*cosa
    twouvca = 2*u*v*cosa
    rt = math.sqrt(u**2 + v**2 + twouvca)
    return ((G*weight)/math.sqrt(1+grade**2.0))*(crr+grade) + 0.5*cda*rho*(upvca)*rt + (cda*rho*u*(u**2 + 0.75*v**2 + twouvca + 0.25* v**2 * math.cos(2*a))) / rt
  return scipy.optimize.fsolve(f, 17, fprime=f_deriv)[0]


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("base", help="The base directory containing experiment parameters.")
  parser.add_argument("-v", "--verbose", help="Enable verbose output.", action="store_true")
  args = parser.parse_args()
  main(args)

