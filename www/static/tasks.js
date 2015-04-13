  var hostname = location.hostname;
    var ros = new ROSLIB.Ros({
      url : 'ws://'+hostname+'/rosws'
    });
  
  function emergency_stop() {
    console.log("notfall");
    var service = new ROSLIB.Service({ros : ros, name : '/go_to_safety_point', serviceType : 'std_srvs/Empty'}); 
    var request = new ROSLIB.ServiceRequest();  
    service.callService(request, function(result) {
      console.log('Called emergency service');
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
            html += "<td>" + task.task_id + "</td>";
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
