  var hostname = location.hostname;
    var ros = new ROSLIB.Ros({
      url : rosws_url
    });
  
  ros.on('connection', function() {
    console.log('Connected to websocket server.');
    $("#connection_broken").addClass('hide');
    $("#connection_ok").removeClass('hide');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    $("#connection_ok").addClass('hide');
    $("#connection_broken").removeClass('hide');
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    $("#connection_ok").addClass('hide');
    $("#connection_broken").removeClass('hide');
  });


  function demand_task(action, waypoint, duration) {
    console.log("demand_task");
    var service = new ROSLIB.Service({ros : ros, name : '/aaf_control_ui_server/demand_task', serviceType : 'aaf_control_ui/DemandTask'}); 
    var request = new ROSLIB.ServiceRequest();
    request.action = action;
    request.waypoint = waypoint;
    request.duration = duration;
    service.callService(request, function(result) {
      console.log('Called demand_task service');
    });
  }

  function clear_schedule() {
    console.log("clear_schedule");
    var service = new ROSLIB.Service({ros : ros, name : '/task_executor/clear_schedule', serviceType : 'std_srvs/Empty'}); 
    var request = new ROSLIB.ServiceRequest();
    service.callService(request, function(result) {
      console.log('Called clear_schedule service: ' + result);
    });
  }


  function init_tasks() {
    var taskTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : '/current_schedule',
        messageType : 'strands_executive_msgs/ExecutionStatus'
    });
    
    taskTopic.subscribe(function(message) {
        // Formats the pose for outputting.
        var text = "not currently in any task"
        
        html = ""
        for (var t=0; t < message.execution_queue.length; t++) {
            var task = message.execution_queue[t];
            var date = new Date(task.execution_time.secs*1000).toLocaleString('de-AT', { timeZone: 'Europe/Vienna' });
            html += "<tr data-toggle=\"modal\" data-target=\"#deletetask\" data-whatever=\" * task.task_id + \">";
            html += "<td>" + task.task_id + "</td>";
            html += "<td>" + task.action + "</td>";
            html += "<td>" + task.start_node_id + "</td>";
            html += "<td>" + date + "</td>";
            html += "</tr>";
           
        }
        console.log(html)

        document.getElementById("tasktext").innerHTML = html;
    });

  }


  function init() {
    init_tasks();
  }
