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


  function init_rosout() {
    var rosoutListener = new ROSLIB.Topic({
      ros : ros,
      name : '/rosout',
      messageType : 'rosgraph_msgs/Log',
    });    
    
    rosoutListener.subscribe(function(message) {
        // Formats the pose for outputting.

        if (message.level > 2 ) {
          var table = document.getElementById("rosout")
          var row = table.insertRow(0);

          var rostime = row.insertCell(0);
          var time = row.insertCell(1);
          var name = row.insertCell(2);
          var msg = row.insertCell(3);
          var file = row.insertCell(4);
          if (message.level==1) color="#444444";
          if (message.level==2) color="#888888";
          if (message.level==4) color="#FF9900";
          if (message.level==8) color="#FF0000";
          if (message.level==16) color="#FF0000";
          row.setAttribute("style", "color: " + color + ";");
          var d = new Date(message.header.stamp.secs * 1000);
          time.innerHTML = d.toLocaleString();
          rostime.innerHTML = message.header.stamp.secs;
          name.innerHTML = message.name;
          file.innerHTML = message.file + ": " + message.line;
          msg.innerHTML = message.msg
        }
    });

  }

  function init_taskevent() {

    task_event_map=[
      "NONE",
      "ADDED",
      "DEMANDED",
      "TASK_STARTED",
      "NAVIGATION_STARTED",
      "NAVIGATION_SUCCEEDED",
      "NAVIGATION_FAILED",
      "NAVIGATION_PREEMPTED",
      "EXECUTION_STARTED",
      "EXECUTION_SUCCEEDED",
      "EXECUTION_FAILED",
      "EXECUTION_PREEMPTED",
      "CANCELLED_MANUALLY",
      "DROPPED",
      "TASK_FINISHED",
      "TASK_FAILED",
      "TASK_SUCCEEDED",
      "TASK_PREEMPTED"
    ];

    var listener = new ROSLIB.Topic({
      ros : ros,
      name : '/task_executor/events',
      messageType : 'strands_executive_msgs/TaskEvent',
    });    
    
    listener.subscribe(function(message) {
        var table = document.getElementById("taskevent")
        var row = table.insertRow(0);

        var event_time = row.insertCell(0);
        var event_name = row.insertCell(1);
        var event_desc = row.insertCell(2);
        var task_id = row.insertCell(3);
        var action = row.insertCell(4);
        var node_id = row.insertCell(5);
        var start = row.insertCell(6);
        var exec_time = row.insertCell(7);
        var end = row.insertCell(8);
        var prio = row.insertCell(9);


        // if (message.level==1) color="#444444";
        // if (message.level==2) color="#888888";
        // if (message.level==4) color="#FF9900";
        // if (message.level==8) color="#FF0000";
        // if (message.level==16) color="#FF0000";
        // row.setAttribute("style", "color: " + color + ";");

        task = message.task

        var et = new Date(message.time.secs * 1000);
        var ds = new Date(task.start_after.secs * 1000);
        var de = new Date(task.end_before.secs * 1000);
        var dx = new Date(task.execution_time.secs * 1000);

        event_time.innerHTML = et.toLocaleString();
        event_desc.innerHTML = message.description;
        exec_time.innerHTML = dx.toLocaleString();
        event_name.innerHTML = message.event + " [" + task_event_map[message.event] + "]";
        task_id.innerHTML = task.task_id;
        action.innerHTML = task.action;
        node_id.innerHTML = task.start_node_id;
        start.innerHTML = ds.toLocaleString();
        end.innerHTML = de.toLocaleString();
        prio.innerHTML = task.priority;
        console.log(message)
    });

  }


  function init() {
    init_rosout();
    init_taskevent();
  }
