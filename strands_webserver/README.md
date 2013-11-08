display_server
==============
To run:

```
rosrun display_server display_server
```

```
roslaunch rosbridge_server rosbridge_websocket.launch 
```

http://localhost:8090

```
rosservice call /display_server/display_page "display_number: 1
content: '<h1>Hello World</h1>'" 
```


# Including other files

The webserver knows about the following includes and process them in this order.

## Twitter Bootstrap

Any request for a file path starting in `bootstrap` will directed to retrieve files relative to the `ros_twitter_bootstrap` ROS package directory.

## Javascript

Any request for a file path end in `.js` will directed to retrieve files relative to the `strands_webtools` ROS package directory.

## Everything else

Anything else not matched by the above rules will be processed relative to the web server root.