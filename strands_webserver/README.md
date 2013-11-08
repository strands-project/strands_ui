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
