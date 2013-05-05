function initialize(coords) {
  // Coords is a list of shmooples
  var center_lat = 0;
  var center_lon = 0;
  for(var i = 0; i < coords.length; i++) {
    center_lat = center_lat + coords[i][0];
    center_lon = center_lon + coords[i][1];
  }
  center_lat = center_lat / coords.length;
  center_lon = center_lon / coords.length;
  var centerlatlng = new google.maps.LatLng(center_lat, center_lon);
  var myOptions = {
		    zoom: 14,
		    center: centerlatlng,
		    mapTypeId: google.maps.MapTypeId.ROADMAP
		  };
  var map = new google.maps.Map(document.getElementById("map_canvas"), myOptions);
  var PolylineCoords = [new google.maps.LatLng(coord[0], coord[1]) for (coord of coords)];

  var Path = new google.maps.Polyline({
    clickable: false,
    geodesic: true,
    path: PolylineCoords,
    strokeColor: "#FF0000",
    strokeOpacity: 1.000000,
    strokeWeight: 2
  });

Path.setMap(map);
}

