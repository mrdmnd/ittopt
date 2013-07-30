from flask import Flask, request, url_for, render_template
import os
import math
import ittoptlib
import requests
DEBUG = False

app = Flask(__name__)
app.config.from_object(__name__)

@app.route('/')
def show_mainpage():
  return render_template('index.html') 

@app.route('/optimize', methods=['POST'])
def optimize():
  try:
    url = request.form["rideurl"]
    model_params = {}
    model_params["resolution"] = float(request.form["resolution"])
    model_params["weight"] = float(request.form["weight"])
    model_params["cda"]  = float(request.form["cda"])
    model_params["crr"] = float(request.form["crr"])
    model_params["rho"] = ittoptlib.airDensity(float(request.form["temp"]), float(request.form["pressure"]), float(request.form["dewpoint"]))
    model_params["wind_direction"] = float(request.form["winddirection"])
    model_params["wind_velocity"] = float(request.form["windvelocity"])
    model_params["power"] = {}
    model_params["power"][30] = float(request.form["30sPower"])
    model_params["power"][60] = float(request.form["1mPower"])
    model_params["power"][300] = float(request.form["5mPower"])
    model_params["power"][600] = float(request.form["10mPower"])
    model_params["power"][1200] = float(request.form["20mPower"])
    model_params["power"][3600] = float(request.form["60mPower"])
  except ValueError:
    return render_template('index.html', console="Try using numerical values, silly.")
  # The meat of the engine.
  course_data = ParseCourseKML(url)
  while True:
    try:
      power, time, dist = RunExperiment(course_data, model_params)
      break
    except ValueError:
      # Catch issues with parameterization
      model_params["resolution"] += 5
  return render_template('index.html', console="Optimal power of %d watts yields time of %.2f seconds (%d:%02d minutes).\nThe course is %f m long, which yields an average speed of %.2f m/s (%.2f km/h)." % (power, time, time/60, time%60, dist, 1.0*dist/time, 3.6*dist/time))
  # Catch issues with parameterization

def ParseCourseKML(url):
  if not url.startswith("http://"):
    url = "http://" + url
  if not url.endswith(".kml"):
    url = url + ".kml"
  content = requests.get(url).content
  started = False
  finished = False
  allPoints = []
  for line in content.splitlines():
    if "<coordinates>" in line:
      started = True
      continue
    if "</coordinates>" in line:
      finished = True
    if started and not finished:
      lon, lat, alt = map(float, line.split(","))
      allPoints.append(ittoptlib.LLAPoint(math.radians(lat), math.radians(lon), alt))
  return allPoints

def RunExperiment(course_data, model):
  # Compress course data
  ecef_course_data = map(ittoptlib.lla2ecef, course_data)
  compression_threshold = model["resolution"]
  compressed_indices = ittoptlib.ramer_douglas_peucker(ecef_course_data, compression_threshold)
  segment_list = ittoptlib.buildSegments(compressed_indices, course_data, model)
  total_dist = 0
  for segment in segment_list:
    total_dist += segment.total_distance
  initial_power_guess = model["power"][3600] * 0.80
  (best_power, best_time) = ittoptlib.chooseOptimalConstantPower(segment_list, model, initial_power_guess)
  return (best_power, best_time, total_dist)

if __name__ == '__main__':
  app.run(debug=DEBUG)

