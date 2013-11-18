



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


/**
Fills the node which matches the jquery selector with a summary of the stats of the current episode.
*/

function writeLatestEpisodeSummary(ros, selector) {
  


    getLatestEpisodeSummary(ros, function(summary) {

        // document.getElementById(id).innerHTML = JSON.stringify(summary);

// {"distance":0,"bump_recoveries":0,"episode_name":"1384810429.189048051","charge_cycles":0,"date":"2013-11-18 21:33:49.189087","run_duration":"0:28:37.061954"}

        var table = '<p>Episode Summary: <table class="table"> <tr> <td>Run time</td><td>{{run_duration}}</td></tr><tr><td>Distance Covered</td> <td>{{distance}}</td></tr> <tr><td>Bumps</td> <td>{{bump_recoveries}}</td></tr> <tr><td>Charge Cycles</td> <td>{{charge_cycles}}</td></tr> </table>';

        var output = Mustache.render(table, summary);
     
        //select the template and transform it
        $(selector).html(output)

        //console.dir(JSON.stringify(summary));
    });


}

