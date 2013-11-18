



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