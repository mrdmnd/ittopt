from flask import Flask, request, render_template, redirect, url_for
import math
import ittoptlib
import requests
DEBUG = True

app = Flask(__name__)
app.config.from_object(__name__)

defaults = {
  'rideurl': 'ridewithgps.com/routes/2831848',
  'weight':'88.0',
  'cda':'0.290',
  'winddirection':'90',
  'windvelocity':'1.5',
  'power60':'563',
  'power180':'402',
  'power300':'363',
  'power600':'320',
  'power1200':'303',
  'power1800':'299',
  'power3600':'275'
}

values = defaults

@app.route('/')
@app.route('/index')
def index():
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
    model_params["wind_direction"] = math.radians(float(request.form["winddirection"]))
    model_params["wind_velocity"] = float(request.form["windvelocity"])
    model_params["power"] = {}
    model_params["power"][60] = float(request.form["1mPower"])
    model_params["power"][180] = float(request.form["3mPower"])
    model_params["power"][300] = float(request.form["5mPower"])
    model_params["power"][600] = float(request.form["10mPower"])
    model_params["power"][1200] = float(request.form["20mPower"])
    model_params["power"][1800] = float(request.form["30mPower"])
    model_params["power"][3600] = float(request.form["60mPower"])
  except ValueError:
    return render_template('index.html', 
      values=values,
      console="Error with some of your inputs. Please sanity check values.")

  # The meat of the engine.
  course_data = ParseCourseKML(url)
  while True:
    try:
      power, time, dist = RunExperiment(course_data, model_params)
      break
    except ValueError:
      #Try a lower resolution
      model_params["resolution"] += 5
  return redirect(url_for('index',
                         rideurl_value_attr='value=%s' % url, 
                         weight_value_attr='value=%.1f' % model_params["weight"],
                         cda_value_attr='value=%.3f' % model_params["cda"],
                         winddirection_value_attr='value=%.5f' % math.degrees(model_params["wind_direction"]),
                         windvelocity_value_attr='value=%.2f' % model_params["wind_velocity"],
                         power60='value=%d' % model_params["power"][60],
                         power180='value=%d' % model_params["power"][180],
                         power300='value=%d' % model_params["power"][300],
                         power600='value=%d' % model_params["power"][600],
                         power1200='value=%d' % model_params["power"][1200],
                         power1800='value=%d' % model_params["power"][1800],
                         power3600='value=%d' % model_params["power"][3600],
                         console="Optimal power of %d watts yields time of %.2f seconds (%d:%02d minutes).\nThe course is %.2f m long, which yields an average speed of %.2f m/s (%.2f km/h)." % (power, time, time/60, time%60, dist, 1.0*dist/time, 3.6*dist/time)))
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