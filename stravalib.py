import json
import urllib
import copy
def geturl(url):
  # Make a request to 
  req = urllib.urlopen(url)
  content = req.read()
  return json.loads(content)


def identify_rideid_of_rank_k(segmentid, rank):
  # 0 is KOM
  resp = geturl("http://app.strava.com/api/v1/segments/%s/efforts?best=true" % str(segmentid))
  efforts = resp['efforts']
  KOM=efforts[rank]
  rideid = KOM["activityId"]
  return rideid

def get_efforts_from_ride(rideid):
  resp = geturl("http://app.strava.com/api/v2/rides/%s/efforts" % str(rideid))
  efforts = resp['efforts']
  return efforts

def get_segment_from_efforts(efforts, segmentid):
  # Find matching 
  # Return a dictionary with keys ["id", "name", "climb_category", "avg_grade", start_latlng", "end_latlng", "elev_difference"
  for effort_elt in efforts:
    # effort = effort_elt['effort']
    segment = effort_elt['segment']
    if segment['id'] == segmentid:
      return segment

def get_stream_latlng(rideid):
  resp = geturl("http://www.strava.com/api/v1/streams/%s" % str(rideid))
  return resp["latlng"]

def get_segment_latlng(all_latlng, segment_details):
  start=segment_details["start_latlng"]
  end=segment_details["end_latlng"]
  start = map(lambda x: round(x, 6), start)
  end = map(lambda x: round(x, 6), end)

  def index_thresholded(tuple_list, value, threshold):
    # Returns the first index i where abs(some_list[i] - value) < threshold.
    # Return -1 if no such i exists.
    (vlat, vlng) = value
    for (i, (lat, lng)) in enumerate(tuple_list):
      if abs(lat - vlat) < threshold and abs(lng - vlng) < threshold:
        return i
    return -1

  start_ind = index_thresholded(all_latlng, start, 0.0001)
  end_ind = index_thresholded(all_latlng, end, 0.0001)
  if start_ind == -1 or end_ind == -1:
    raise Exception("could not find segment endpoints in data")
  else:
    return filter(lambda x: x != [0.0, 0.0], all_latlng[start_ind:min(end_ind+1, len(all_latlng))])

def _vec2d_dist(p1, p2):
  return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

def _vec2d_sub(p1, p2):
  return (p1[0] - p2[0], p1[1]-p2[1])

def _vec2d_mul(p1, p2):
  return p1[0]*p2[0] + p1[1]*p2[1]

def ramerdouglas(line, dist=0.0001):
  if len(line) < 3:
    return line

  begin, end = line[0], line[-1]
  distSq = []
  for curr in line[1:-1]:
    tmp = (_vec2d_dist(begin, curr) - _vec2d_mul(_vec2d_sub(end, begin), _vec2d_sub(curr, begin)) ** 2 / _vec2d_dist(begin, end))
    distSq.append(tmp)
  maxdist = max(distSq)
  if maxdist < dist ** 2:
    return [begin, end]
  pos = distSq.index(maxdist)
  return (ramerdouglas(line[:pos + 2], dist) + ramerdouglas(line[pos + 1:], dist)[1:]) 

def append_elevation(latlng_list):
  path = "|".join([str(pt[0])+","+str(pt[1]) for pt in latlng_list])
  samples = str(len(latlng_list))
  url = "http://maps.googleapis.com/maps/api/elevation/json?path="+path+"&samples="+samples+"&sensor=false"
  resp = geturl(url)
  if not resp["status"] == "OK":
    raise Exception("Google Maps API failed to return elevation data")
  results = resp["results"]
  latlngalt = copy.deepcopy(latlng_list)
  for index, result in enumerate(results):
    latlngalt[index].append(float(result["elevation"]))
  return latlngalt

if __name__ == "__main__":
  whatever_segment_id = 3255314
  KOM_ride_id = identify_rideid_of_rank_k(whatever_segment_id, 0)
  efforts_on_that_ride = get_efforts_from_ride(KOM_ride_id)
  segment_details = get_segment_from_efforts(efforts_on_that_ride, whatever_segment_id)

  all_stream_latlng = get_stream_latlng(KOM_ride_id)
  segment_latlng = get_segment_latlng(all_stream_latlng, segment_details)

  compressed_polyline_points = ramerdouglas(segment_latlng)
  print len(all_stream_latlng)
  print len(segment_latlng)
  print len(compressed_polyline_points)
  print compressed_polyline_points
