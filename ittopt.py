from flask import Flask, request, url_for, render_template 
from stravalib import *
DEBUG = True

app = Flask(__name__)
app.config.from_object(__name__)

@app.route('/')
def show_mainpage():
  return render_template('index.html') 

@app.route('/optimize', methods=['POST'])
def optimize():
  url = request.form["rideurl"]
  try:
    segmentid = int(url[url.rindex('/')+1:])
    weight = float(request.form["weight"])
    cda = float(request.form["frontalcda"])
    wind_direction = float(request.form["winddirection"])
    wind_velocity = float(request.form["windvelocity"])
    resolution = float(request.form["resolution"])
    power_30 = float(request.form["30sPower"])
    power_60 = float(request.form["1mPower"])
    power_300 = float(request.form["5mPower"])
    power_600 = float(request.form["10mPower"])
    power_1200 = float(request.form["20mPower"])
    power_3600 = float(request.form["60mPower"])
  except ValueError:
    return render_template('index.html', console="Try using numerical values, silly.")
  
  rank = 0
  while True:
    if rank > 6:
      return render_template('index.html', console="Unable to obtain segment data after checking top 6 riders, perhaps something is borked?")
    try:
      rideid = identify_rideid_of_rank_k(segmentid, rank)
      efforts = get_efforts_from_ride(rideid)
      segment_details = get_segment_from_efforts(efforts, segmentid)
      full_stream_latlng = get_stream_latlng(rideid)
      segment_latlng = get_segment_latlng(full_stream_latlng, segment_details)
      compressed_polyline = ramerdouglas(segment_latlng, dist=resolution)
      latlngalt = append_elevation(compressed_polyline)
      break
    except Exception:
      print "Bad data on rider of rank %d" % rank
      rank += 1
      continue
  #except Exception:
  #  return render_template('index.html', console="An unexpected error occurred while trying to load data for this segment.")
  if len(latlngalt) <= 2:
    return render_template('index.html', console="Unable to obtain compressed segment data.")
  
  else:
    # Do the rendering on a new page
    line_segs = ""
    for i in range(len(latlngalt)-1):
      line_segs += "%s --> %s \n" % (str(latlngalt[i]), str(latlngalt[i+1]))
    
    return render_template('results.html', console=line_segs, mapcoords=str(compressed_polyline))


if __name__ == '__main__':
  app.run(debug=True)

