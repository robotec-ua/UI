var NAV2D = NAV2D || { REVISION: "0.3.0" };

var count = null;
var robotvisible = false;
var may_set_point = false;

let goalMarker = new Array();
var goalMarkerCounter = 0;

NAV2D.OccupancyGridClientNav = function (options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || "/map";
  var continuous = options.continuous;
  this.serverName = options.serverName || "/move_base";
  this.actionName = options.actionName || "move_base_msgs/MoveBaseAction";
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;
  this.withOrientation = options.withOrientation || true;
  this.navigator = null;

  // setup a client to get the map
  var client = new ROS2D.OccupancyGridClient({
    ros: this.ros,
    rootObject: this.rootObject,
    continuous: continuous,
    topic: topic,
  });

  client.on("change", function () {
    that.navigator = new NAV2D.Navigator({
      ros: that.ros,
      serverName: that.serverName,
      actionName: that.actionName,
      rootObject: that.rootObject,
      withOrientation: that.withOrientation,
    });

    // scale the viewer to fit the map
    that.viewer.scaleToDimensions(
      client.currentGrid.width,
      client.currentGrid.height
    );
    that.viewer.shift(
      client.currentGrid.pose.position.x,
      client.currentGrid.pose.position.y
    );
  });
};

NAV2D.Navigator = function (options) {
  console.log("Navigator");
  var that = this;
  options = options || {};

  var ros = options.ros;
  this.rootObject = options.rootObject || new createjs.Container();

  var message_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/ui_message",
    messageType: "std_msgs/String",
  });

  message_listener.publish(
    new ROSLIB.Message({ data: "Initialization completed" })
  );

  var WP_array = new ROSLIB.Topic({
    ros: ros,
    name: "/WPs_topic",
    messageType: "geometry_msgs/PoseArray",
  });

  var WP_req = new ROSLIB.Topic({
    ros: ros,
    name: "/WP_req",
    messageType: "std_msgs/Empty",
  });

  WP_array.subscribe(function (data) {
    goalMarkerCounter = 0;

    for (let i = 0; i < data.poses.length; i++) {
      goalMarker[goalMarkerCounter] = new ROS2D.NavigationArrow({
        size: 15,
        strokeSize: 1,
        fillColor: createjs.Graphics.getRGB(255, 0, 0, 1),
        pulse: false,
      });

      goalMarker[goalMarkerCounter].x =
        data.poses[goalMarkerCounter].position.x;
      goalMarker[goalMarkerCounter].y =
        -data.poses[goalMarkerCounter].position.y;
      goalMarker[goalMarkerCounter].rotation = stage.rosQuaternionToGlobalTheta(
        data.poses[goalMarkerCounter].orientation
      );
      goalMarker[goalMarkerCounter].scaleX = 1.0 / stage.scaleX;
      goalMarker[goalMarkerCounter].scaleY = 1.0 / stage.scaleY;
      that.rootObject.addChild(goalMarker[goalMarkerCounter]);
      goalMarkerCounter++;
    }
  });

  var may_set_point = new ROSLIB.Topic({
    ros: ros,
    name: "/may_set_point",
    messageType: "std_msgs/Bool",
  });

  may_set_point.subscribe(function (data) {
    may_set_point = data.data;
    if (may_set_point === true) {
      for (let i = 0; i < goalMarker.length; i++) {
        that.rootObject.removeChild(goalMarker[i]);
      }
      goalMarkerCounter = 0;
    }
  });

  function way_point_func(pose) {
    console.log("way_point_func");
    var way_point = new ROSLIB.Topic({
      ros: ros,
      name: "/new_way_point",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    var pose = new ROSLIB.Message({
      header: { frame_id: "map" },
      pose: {
        pose: {
          position: { x: pose.position.x, y: pose.position.y, z: 0.0 },
          orientation: { z: pose.orientation.z, w: pose.orientation.w },
        },
        covariance: [
          0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942,
        ],
      },
    });
    way_point.publish(pose);
  }

  var stage;

  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }

  // marker for the robot
  var robotMarker = new ROS2D.NavigationArrow({
    size: 25,
    strokeSize: 1,
    fillColor: createjs.Graphics.getRGB(0, 0, 255, 1),
    pulse: false,
  });
  robotMarker.visible = false;

  var initScaleSet = false;
  // setup a listener for the robot pose
  var poseListener = new ROSLIB.Topic({
    ros: ros,
    name: "/robot_pose",
    messageType: "geometry_msgs/Pose",
    throttle_rate: 1,
  });

  that.rootObject.addChild(robotMarker);

  poseListener.subscribe(function (pose) {
    // update the robots position on the map
    robotMarker.x = pose.position.x;
    robotMarker.y = -pose.position.y;
    console.log("robotMarker");
    if (!initScaleSet) {
      robotMarker.scaleX = 1.0 / stage.scaleX;
      robotMarker.scaleY = 1.0 / stage.scaleY;
      initScaleSet = true;
    }
    // change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
    robotMarker.visible = true;
  });

  var position = null;
  var positionVec3 = null;
  var thetaRadians = 0;
  var thetaDegrees = 0;
  var orientationMarker = null;
  var mouseDown = false;
  var mouseMove = false;
  var xDelta = 0;
  var yDelta = 0;

  var pannavfunction = function (event, mouseState) {
    if (may_set_point == true) {
      if (mouseState === "down") {
        // get position when mouse button is pressed down
        position = stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new ROSLIB.Vector3(position);

        mouseDown = true;
      } else if (mouseState === "move") {
        mouseMove = true;
        // remove obsolete orientation marker
        that.rootObject.removeChild(orientationMarker);

        if (mouseDown === true) {
          var currentPos = stage.globalToRos(event.stageX, event.stageY);
          var currentPosVec3 = new ROSLIB.Vector3(currentPos);

          orientationMarker = new ROS2D.NavigationArrow({
            size: 25,
            strokeSize: 1,
            fillColor: createjs.Graphics.getRGB(0, 255, 0, 1),
            pulse: false,
          });

          xDelta = currentPosVec3.x - positionVec3.x;
          yDelta = currentPosVec3.y - positionVec3.y;

          thetaRadians = Math.atan2(xDelta, yDelta);

          thetaDegrees = thetaRadians * (180.0 / Math.PI);

          if (thetaDegrees >= 0 && thetaDegrees <= 180) {
            thetaDegrees += 270;
          } else {
            thetaDegrees -= 90;
          }
          orientationMarker.x = positionVec3.x;
          orientationMarker.y = -positionVec3.y;
          orientationMarker.rotation = thetaDegrees;
          orientationMarker.scaleX = 1.0 / stage.scaleX;
          orientationMarker.scaleY = 1.0 / stage.scaleY;
          that.rootObject.addChild(orientationMarker);
        }
      } else if (mouseState === "up") {
        if (mouseDown) {
          if (mouseMove) {
            mouseMove = false;
            goalMarker[goalMarkerCounter] = new ROS2D.NavigationArrow({
              size: 15,
              strokeSize: 1,
              fillColor: createjs.Graphics.getRGB(255, 0, 0, 1),
              pulse: false,
            });

            goalMarker[goalMarkerCounter].x = orientationMarker.x;
            goalMarker[goalMarkerCounter].y = orientationMarker.y;
            goalMarker[goalMarkerCounter].rotation = orientationMarker.rotation;
            goalMarker[goalMarkerCounter].scaleX = orientationMarker.scaleX;
            goalMarker[goalMarkerCounter].scaleY = orientationMarker.scaleY;
            that.rootObject.addChild(goalMarker[goalMarkerCounter]);

            goalMarkerCounter++;

            mouseDown = false;

            var goalPos = stage.globalToRos(event.stageX, event.stageY);
            var goalPosVec3 = new ROSLIB.Vector3(goalPos);

            xDelta = goalPosVec3.x - positionVec3.x;
            yDelta = goalPosVec3.y - positionVec3.y;

            thetaRadians = Math.atan2(xDelta, yDelta);

            if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
              thetaRadians += (3 * Math.PI) / 2;
            } else {
              thetaRadians -= Math.PI / 2;
            }
            var qz = Math.sin(-thetaRadians / 2.0);
            var qw = Math.cos(-thetaRadians / 2.0);

            var orientation = new ROSLIB.Quaternion({
              x: 0,
              y: 0,
              z: qz,
              w: qw,
            });

            var pose = new ROSLIB.Pose({
              position: positionVec3,
              orientation: orientation,
            });

            way_point_func(pose);
            that.rootObject.removeChild(orientationMarker);
          } else {
            message_listener.publish(
              new ROSLIB.Message({
                data: "Please, set the direction of the WayPoint",
              })
            );
          }
        }
      }
    }
  };

  WP_req.publish();
  
  const funcMove = (event) => {
    console.log("move");
    pannavfunction(event, "move");
  };

  this.rootObject.addEventListener("stagemousedown", function (event) {
    console.log("down");
    pannavfunction(event, "down");
    that.rootObject.addEventListener("stagemousemove", funcMove);
  });

  this.rootObject.addEventListener("stagemouseup", function (event) {
    console.log("up");
    pannavfunction(event, "up");
    that.rootObject.removeEventListener("stagemousemove", funcMove);
  });
};
