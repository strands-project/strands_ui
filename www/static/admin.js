  var hostname = location.hostname;
    var ros = new ROSLIB.Ros({
      url : rosws_url
    });
  

  function init_rosout() {
    var rosoutListener = new ROSLIB.Topic({
      ros : ros,
      name : '/rosout',
      messageType : 'rosgraph_msgs/Log',
    });    
    
    rosoutListener.subscribe(function(message) {
        // Formats the pose for outputting.

        if (message.level >=2 ) {
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
          console.log(message)
        }
        //document.getElementById("batterytext").update(battery + "%");
    });

  }


  function init() {
    init_rosout();
  }
