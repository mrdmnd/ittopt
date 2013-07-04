import argparse
import glob
import os
import numpy

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
    print "First point: %s" % str(lat_lon_alt[0])
    print "Second point: %s" % str(lat_lon_alt[-1])
  segment_list = buildSegments(lat_lon_alt)


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


def buildSegments(lat_lon_alt):
  # For each adjacent lat_lon_alt point, constructs a segment between them that tracks the 
  pass

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("base", help="The base directory containing experiment parameters.")
  parser.add_argument("-v", "--verbose", help="Enable verbose output.", action="store_true")
  args = parser.parse_args()
  main(args)

