
  function init_say(ros) {
    var sayTopic = new ROSLIB.Topic({
        ros         : ros,
        name        : '/mary_tts/speak',
        messageType : 'std_msgs/String'
    });
    
    sayTopic.subscribe(function(message) {
        // Formats the pose for outputting.
        var say = message.data;

        document.getElementById("saytext").value=say;
    });

  }

  function init_battery(ros) {
    var batteryListener = new ROSLIB.Topic({
      ros : ros,
      name : '/battery_state',
      messageType : 'scitos_msgs/BatteryState',
      throttle_rate : 1
    });    
    
    batteryListener.subscribe(function(message) {
        // Formats the pose for outputting.
        var battery = message.lifePercent;

        document.getElementById("batterytext").value=battery + "%";
    });

  }

  function init_mjpeg(hostname) {
    // Create the main viewer.
    var viewer = new MJPEGCANVAS.Viewer({
    divID : 'mjpeg',
    host : hostname,
    width : 320,
    height : 240,
    topic : '/image'
    });
 
  }



  function init() {
    var hostname = location.hostname;
    // Connecting to ROS.

    init_mjpeg(hostname)

    var ros = new ROSLIB.Ros({
      url : 'ws://'+hostname+':9090'
    });
    
    init_say(ros)
    init_battery(ros)
    

    // Create the main viewer.
    var viewer = new ROS2D.Viewer({
      divID : 'nav',
      width : 600,
      height : 400
    });

    // Subscribes to the robot's OccupancyGrid, which is ROS representation of
    // the map, and renders the map in the scene.
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene
    });
    gridClient.on('change', function() {
    // scale the viewer to fit the map
    viewer.scaleToDimensions(gridClient.currentGrid.width, 
      gridClient.currentGrid.height);
    viewer.shift(gridClient.currentGrid.x, gridClient.currentGrid.y-21);
    // get a handle to the stage
    var stage;
    if (viewer.scene instanceof createjs.Stage) {
      stage = viewer.scene;
    } else {
      stage = viewer.scene.getStage();
    }
    // marker for the robot
    var robotMarker = new ROS2D.NavigationArrow({
      size : 8,
      strokeSize : 1,
      fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),
      pulse : false
    });
    // wait for a pose to come in first
    robotMarker.visible = false;
   
    viewer.scene.addChild(robotMarker);
    var initScaleSet = false;
    // setup a listener for the robot pose
    var poseListener = new ROSLIB.Topic({
      ros : ros,
      name : '/robot_pose',
      messageType : 'geometry_msgs/Pose',
      throttle_rate : 10
    });
    poseListener.subscribe(function(pose) {
      // update the robots position on the map
      robotMarker.x = pose.position.x;
      robotMarker.y = -pose.position.y;
      
      if (!initScaleSet) {
        robotMarker.scaleX = 1.0 / stage.scaleX;
        robotMarker.scaleY = 1.0 / stage.scaleY;
        initScaleSet = true;
      }
      // change the angle
      robotMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
      robotMarker.visible = true;
    });
   });
  }
