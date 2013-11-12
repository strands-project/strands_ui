# STRANDS User Interfaces

User interfaces for the robots, displayed using a web browser.

# Requirements

`rosdep` should give you most requirements, but we don't yet have a rosdep for [web.py](http://webpy.org) which is required by `strands_webserver`. This can be installed using, e.g., `sudo pip install web.py`


# `strands_webserver`

The `strands_webserver` is a node which both acts as a webserver and a ROS node which can receive input from other nodes. This allows these other nodes to control what the webserver displays and receive feedback from user interaction with the pages.

## Running

To run the server, first run rosbrige if you don't have it running already.
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
Then run the server
```bash
rosrun strands_webserver strands_webserver 
```

This will launch the webserver on localhost port 8090, so browse to [http://localhost:8090](http://localhost:8090) and you should see something like
```text
Ready; display = 1
```

## Displaying content

The webserver manages individual *displays* separately, one for each request to its URL. These can be published to individual by specifying their number, or you can just send 0 to publish to all of them.

Interaction with `strands_webserver` typically happens by publishing instructions to it for display. These can either be URLs or bits of html which will be injected into its main page. 
