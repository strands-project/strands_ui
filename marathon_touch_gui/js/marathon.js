



/**
Get current episode summary 
*/
function getLatestEpisodeName(ros, fn) {
 
      var service = new ROSLIB.Service({
        ros : ros,
        name : '/ap_log_server/get_latest_episode_name',
        serviceType : 'ap_msgs/GetLatestEpisodeName'
      });

      var request = new ROSLIB.ServiceRequest({});

      service.callService(request, function(result) {
      	fn(result.name);
      });


}

/**
Get current episode summary in JSON
*/
function getLatestEpisodeSummary(ros, fn) {

    getLatestEpisodeName(ros, function(ep) {

      var service = new ROSLIB.Service({
        ros : ros,
        name : '/ap_log_server/get_episode_summary',
        serviceType : 'ap_msgs/GetEpisodeSummary'
      });

      var request = new ROSLIB.ServiceRequest({episode_name: ep});

      service.callService(request, function(result) {    
      	fn(jQuery.parseJSON(result.summary));
      });    

    }

);

}

// function substringToCS(string, char) {
//   return string.substring(0,string.indexOf())
// }

/**
Fills the node which matches the jquery selector with a summary of the stats of the current episode.
*/

function writeLatestEpisodeSummary(ros, selector) {
  
    getLatestEpisodeSummary(ros, function(summary) {

        // document.getElementById(id).innerHTML = JSON.stringify(summary);

// 

// {"distance":0,"failed_waypoints":0,"bump_recoveries":0,"successful_waypoints":0,"episode_name":"1384875846.717848062","charge_cycles":0,"active_waypoint":"charging_point","date":"2013-11-19 15:44:06.717880","run_duration":"0:00:01.006879","navigation_recoveries":0}


        var view = {
            run_duration: summary.run_duration.substring(0,summary.run_duration.indexOf(".")),
            distance: summary.distance,
            bump_recoveries: summary.bump_recoveries,
            charge_cycles: summary.charge_cycles,
            waypoint_attempts: summary.successful_waypoints + summary.failed_waypoints,
            waypoint_successess: summary.successful_waypoints,
            active_waypoint: summary.active_waypoint,
            navigation_recoveries: summary.navigation_recoveries
          };


        var table = '<table class="table"> <tr> <td>Run time</td><td>{{run_duration}} <em>hours</em></td></tr><tr><td>Distance Covered</td> <td>{{distance}} <em>m</em></td></tr> <tr><td>Current Target</td> <td>{{active_waypoint}}</td></tr> <tr><td>Waypoints Reached</td> <td>{{waypoint_successess}}/{{waypoint_attempts}}</td></tr> <tr><td>Bump Recoveries</td> <td>{{bump_recoveries}}</td></tr> <tr><td>Navigation Recoveries</td> <td>{{navigation_recoveries}}</td></tr> <tr><td>Charge Cycles</td> <td>{{charge_cycles}}</td></tr> </table>';
        
        var output = Mustache.render(table, view);
     
        //select the template and transform it
        $(selector).html(output)

        console.dir(JSON.stringify(summary));
    });


}

function writeBatteryLevelFromTemplate(ros, selector, template) {
  
// setup a listener for the robot pose
        var batteryListener = new ROSLIB.Topic({
          ros : ros,
          name : '/battery_state',
          messageType : 'scitos_msgs/BatteryState',
          throttle_rate : 1
        });

        batteryListener.subscribe(function(batteryState) {

          var view = {
            level: batteryState.lifePercent
          };
          var output = Mustache.render(template, view);
          $(selector).html(output)

        })  ;
       
}

/**
Fills the node which matches the jquery selector with a readout of the battery level.
*/

function writeBatteryLevel(ros, selector) {
  
  writeBatteryLevelFromTemplate(ros, selector, "<p>Battery level: {{level}}</p>");
       
}
