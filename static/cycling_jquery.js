$(document).ready(function() {

  $("#advanced_params").hide();

  $("#toggle_optional").click( function() {
    $("#advanced_params").toggle(400);
  });
  $("#rideurl").blur( function() {
    ride_url = $("#rideurl").val()
    $("#map_img").attr("src", ride_url+"/full.png").fadeIn(200);
    $("#elev_img").attr("src", ride_url+"/elevation_profile").fadeIn(200);
  });
});
