from flask import Flask, request, render_template, redirect, url_for, session, flash
import os
import math
import ittoptlib
import requests
DEBUG = True

app = Flask(__name__)
app.config.from_object(__name__)
app.secret_key = os.urandom(24)

#Set defaults
defaults = {
  'rideurl': 'ridewithgps.com/routes/2831848',
  'weight':'88.0',
  'cda':'0.290',
  'wind-direction':'90',
  'wind-velocity':'1.5',
  '1mPower':'563',
  '3mPower':'402',
  '5mPower':'363',
  '10mPower':'320',
  '20mPower':'303',
  '30mPower':'299',
  '60mPower':'275'
}

form_model_names = {
  'resolution':'resolution',
  'weight':'weight',
  'cda':'cda',
  'crr':'crr',
  'wind-direction':'wind_direction',
  'wind-velocity':'wind_velocity'
}

form_power_names = {
  '1mPower':60,
  '3mPower':180,
  '5mPower':300,
  '10mPower':600,
  '20mPower':1200,
  '30mPower':1800,
  '60mPower':3600,
}

@app.route('/')
@app.route('/index')
def index():
  return render_template('index.html')

@app.route('/optimize', methods=['POST'])
def optimize():
  try:
    url = request.form["rideurl"]
    model_params = {}
    
    #Map the form values into our model params
    for form_name, param_name in form_model_names.iteritems():
      value = float(request.form[form_name])
      session[form_name] = value
      model_params[param_name] = value

    #Fix some math
    model_params['wind_direction'] = math.radians(model_params['wind_direction'])
    temp, pressure, dew = float(request.form['temp']), float(request.form['pressure']), float(request.form['dewpoint'])
    model_params['rho'] = ittoptlib.airDensity(temp, pressure, dew)
    session['temp'], session['pressure'], session['dewpoint'] = temp, pressure, dew
    
    #Map the power values into the parameters package
    model_params['power'] = {}
    for form_name, power in form_power_names.iteritems():
      value = float(request.form[form_name])
      model_params['power'][power] = value
      session[form_name] = value

  except ValueError:
    flash('Error with some of your inputs. Please sanity check values.')
    return redirect(url_for('index'))

  # The meat of the engine.
  course_data = ParseCourseKML(url)
  while True:
    try:
      power, time, dist = RunExperiment(course_data, model_params)
      break
    except ValueError:
      #Try a lower resolution
      model_params["resolution"] += 5
  session['console'] = "Optimal power of %d watts yields time of %.2f seconds (%d:%02d minutes).\nThe course is %.2f m long, which yields an average speed of %.2f m/s (%.2f km/h)." % (power, time, time/60, time%60, dist, 1.0*dist/time, 3.6*dist/time)
  return redirect(url_for('index'))

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