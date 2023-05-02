$(document).ready(function() {
  $('[title!=""]').qtip({
    position: {
      my: 'right center',
      at: 'left center',
    }, 
    style: {
      classes: 'myQtipClass qtip-dark qtip-shadow qtip-rounded'
    }
  });

  $("#advanced_params").hide();
  $("#map_img").hide();
  $("#elev_img").hide();

  $("#toggle_optional").click( function() {
    $("#advanced_params").toggle(400);
  });
  $("#rideurl").blur( function() {
    ride_url = $("#rideurl").val();
    if (ride_url.substring(0, 7) != "http://") {
      ride_url = "http://" + ride_url;

    }
    $("#map_img").attr("src", ride_url+"/full.png").show(400);
    $("#elev_img").attr("src", ride_url+"/elevation_profile").show(400);
  });
});
