from flask import Flask, request, url_for, render_template
import os
import ittoptlib
DEBUG = True

app = Flask(__name__)
app.config.from_object(__name__)

@app.route('/')
def show_mainpage():
  return render_template('index.html') 

@app.route('/optimize', methods=['POST'])
def optimize():
  try:
    url = request.form["rideurl"]
    model_params["num_waypoints"] = float(request.form["resolution"])
    model_params = {}
    model_params["rider"] = {}
    model_params["rider"]["weight"] = float(request.form["weight"])
    model_params["rider"]["cda"]  = float(request.form["cda"])
    model_params["rider"]["crr"] = float(request.form["crr"])
    model_params["environ"] = {}
    model_params["environ"]["temp"] = float(request.form["temp"])
    model_params["environ"]["dewpoint"] = float(request.form["dewpoint"])
    model_params["environ"]["pressure"] = float(request.form["pressure"])
    model_params["environ"]["wind_direction"] = float(request.form["winddirection"])
    model_params["environ"]["wind_velocity"] = float(request.form["windvelocity"])
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
  results = RunExperiment(course_data, model_params)
  return RenderResults(results)

def ParseCourseKML(url):
  content = urllib2.request(url)

def RenderResults(results):
  return render_template('results.html', console="Foobar!", mapcoords=dsfho)

if __name__ == '__main__':
  app.run(debug=True)

