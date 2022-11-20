let zoom_storage = localStorage.getItem("my_zoom_storage");

if (zoom_storage == null) {
  zoom_storage = 0;
} else {
  zoom_storage = parseInt(zoom_storage);
}

window.onload = function () {
  document.getElementsByClassName("video-streaming")[0].src =
    "http://192.168.0.103:8090/stream_viewer?topic=/raspicam_node/image&type=ros_compressed";

  const ros = new ROSLIB.Ros({
    url: "ws://192.168.0.103:9092",
  });

  ros.on("connection", function () {
    console.log("Connected to websocket server.");
  });
  ros.on("error", function (error) {
    console.log("Error connecting to websocket server: ", error);
  });
  ros.on("close", function () {
    console.log("Connection to websocket server closed.");
  });

  const viewer = new ROS2D.Viewer({
    divID: "nav",
    width: 600,
    height: 400,
  });

  const nav = NAV2D.OccupancyGridClientNav({
    ros: ros,
    rootObject: viewer.scene,
    viewer: viewer,
    continuous: true,
  });

  window.zoomInMap = function (ros, viewer) {
    const zoom = new ROS2D.ZoomView({
      ros: ros,
      rootObject: viewer.scene,
    });
    zoom.startZoom(300, 200);
    zoom.zoom(1.2);
  };

  window.zoomOutMap = function (ros, viewer) {
    const zoom = new ROS2D.ZoomView({
      ros: ros,
      rootObject: viewer.scene,
    });
    zoom.startZoom(300, 200);
    zoom.zoom(0.8);
  };

  const may_set_point = new ROSLIB.Topic({
    ros: ros,
    name: "/may_set_point",
    messageType: "std_msgs/Bool",
  });

  const cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/Twist",
  });

  const uiOperation = new ROSLIB.Topic({
    ros: ros,
    name: "/ui_operation",
    messageType: "std_msgs/String",
  });

  const message_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/ui_message",
    messageType: "std_msgs/String",
  });

  const velocity_listener = new ROSLIB.Topic({
    ros: ros,
    name: "/raw_vel",
    messageType: "geometry_msgs/Twist",
  });

  // -------------------launcher------------------
  const buildMapMessage = new ROSLIB.Message({ data: "build_map" });
  const saveMapMessage = new ROSLIB.Message({ data: "save_map" });
  const homeMessage = new ROSLIB.Message({ data: "home" });
  const brakeMessage = new ROSLIB.Message({ data: "brake" });

  const func1OnMessage = new ROSLIB.Message({ data: "func1_on" });
  const func1OffMessage = new ROSLIB.Message({ data: "func1_off" });
  const func2OnMessage = new ROSLIB.Message({ data: "func2_on" });
  const func2OffMessage = new ROSLIB.Message({ data: "func2_off" });

  // ---------------------Follow_wp--------------------
  const followMessage = new ROSLIB.Message({ data: "follow" });
  const saveWPMessage = new ROSLIB.Message({ data: "save_wp" });
  const clearMessage = new ROSLIB.Message({ data: "clear" });
  const previousMessage = new ROSLIB.Message({ data: "previous_wp" });
  const nextMessage = new ROSLIB.Message({ data: "next_wp" });

  const zoomInButton = document.getElementById(
    "head-control-button-center-zoom-in"
  );
  const zoomOutButton = document.getElementById(
    "head-control-button-center-zoom-out"
  );

  const forwardButton = document.getElementById("button-forward");
  const leftButton = document.getElementById("button-left");
  const rightButton = document.getElementById("button-right");
  const backwardButton = document.getElementById("button-backward");
  const followButton = document.getElementById("button-center");

  const buildMapButton = document.getElementById("head-control-button-left");
  const saveMapButton = document.getElementById("head-control-button-center-map");
  const homeButton = document.getElementById("head-control-button-home");
  const brakeButton = document.getElementById("head-control-button-brake");
  const previousButton = document.getElementById("head-control-button-previous-wp");
  const nextButton = document.getElementById("head-control-button-next-wp");
  const saveWpButton = document.getElementById("head-control-button-right");
  const clearButton = document.getElementById("head-control-button-center-clear");

  const func1Slider = document.getElementById("head-func1-slider");
  func1Slider.addEventListener("change", func1SliderValueChanged, false);

  function func1SliderValueChanged() {
    const text = document.getElementById("statistics__desc").innerHTML;
    document.getElementById("statistics__desc__previous").innerHTML = text;
    if (func1Slider.value === "0") {
      document.getElementById(
        "statistics__desc"
      ).innerHTML = `[INFO]: Function 1 OFF`;
      uiOperation.publish(func1OffMessage);
    } else {
      document.getElementById(
        "statistics__desc"
      ).innerHTML = `[INFO]: Function 1 ON`;
      uiOperation.publish(func1OnMessage);
    }
  }

  const func2Slider = document.getElementById("head-func2-slider");
  func2Slider.addEventListener("change", func2SliderValueChanged, false);

  function func2SliderValueChanged() {
    const text = document.getElementById("statistics__desc").innerHTML;
    document.getElementById("statistics__desc__previous").innerHTML = text;
    if (func2Slider.value === "0") {
      document.getElementById(
        "statistics__desc"
      ).innerHTML = `[INFO]: Function 1 OFF`;
      uiOperation.publish(func2OffMessage);
    } else {
      document.getElementById(
        "statistics__desc"
      ).innerHTML = `[INFO]: Function 1  ON`;
      uiOperation.publish(func2OnMessage);
    }
  }

  let linear_сount = 0,
    angular_count = 0;

  let linear_acc = 0.008,
    linear_dec = 0.005,
    linear_max = 0.2;

  let angular_acc = 0.08,
    angular_dec = 0.08,
    angular_max = 0.6;

  document.addEventListener("keydown", (event) => {
    if (
      event.keyCode == 87 ||
      event.keyCode == 38 ||
      event.keyCode == 83 ||
      event.keyCode == 40
    ) {
      /* w */ /* up */ /* s */ /* down */
      if (event.keyCode == 83 || event.keyCode == 40) {
        /* s */ /* down */
        if (linear_сount > -linear_max) {
          linear_сount -= linear_acc;
        } else linear_сount = -linear_max;
        message_listener.publish(
          new ROSLIB.Message({ data: "Moving BACKWARD" })
        );
      } else if (event.keyCode == 87 || event.keyCode == 38) {
        /* w */ /* up */
        if (linear_сount < linear_max) {
          linear_сount += linear_acc;
        } else linear_сount = linear_max;
        message_listener.publish(
          new ROSLIB.Message({ data: "Moving FORWARD" })
        );
      }
      cmdVel.publish(
        new ROSLIB.Message({ linear: { x: linear_сount, y: 0, z: 0 } })
      );
    }
    if (
      event.keyCode == 65 ||
      event.keyCode == 37 ||
      event.keyCode == 68 ||
      event.keyCode == 39
    ) {
      /* a */ /* left */ /* d */ /* right */
      if (event.keyCode == 65 || event.keyCode == 37) {
        /* a */ /* left */
        if (angular_count < angular_max) {
          angular_count += angular_acc;
        } else angular_count = angular_max;
        message_listener.publish(new ROSLIB.Message({ data: "Moving LEFT" }));
      } else if (event.keyCode == 68 || event.keyCode == 39) {
        /* d */ /* right */
        if (angular_count > -angular_max) {
          angular_count -= angular_acc;
        } else angular_count = -angular_max;
        message_listener.publish(new ROSLIB.Message({ data: "Moving RIGHT" }));
      }
      cmdVel.publish(
        new ROSLIB.Message({ angular: { x: 0, y: 0, z: angular_count } })
      );
    }
  });

  document.addEventListener("keyup", (event) => {
    if (
      event.keyCode == 87 ||
      event.keyCode == 38 ||
      event.keyCode == 83 ||
      event.keyCode == 40
    ) {
      /* w */ /* up */ /* s */ /* down */

      if (event.keyCode == 87 || event.keyCode == 38) {
        forward_stop_interval = setInterval(function () {
          if (linear_сount > 0) {
            linear_сount -= linear_dec;
          } else {
            linear_сount = 0;
            clearInterval(forward_stop_interval);
          }
          message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
          cmdVel.publish(
            new ROSLIB.Message({ linear: { x: linear_сount, y: 0, z: 0 } })
          );
        }, 25);
      } else if (event.keyCode == 83 || event.keyCode == 40) {
        backward_stop_interval = setInterval(function () {
          if (linear_сount < 0) {
            linear_сount += linear_dec;
          } else {
            linear_сount = 0;
            clearInterval(backward_stop_interval);
          }
          message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
          cmdVel.publish(
            new ROSLIB.Message({ linear: { x: linear_сount, y: 0, z: 0 } })
          );
        }, 25);
      }
    }
    if (
      event.keyCode == 65 ||
      event.keyCode == 37 ||
      event.keyCode == 68 ||
      event.keyCode == 39
    ) {
      /* a */ /* left */ /* d */ /* right */

      if (event.keyCode == 65 || event.keyCode == 37) {
        /* a */ /* left */
        left_stop_interval = setInterval(function () {
          if (angular_count > 0) {
            angular_count -= angular_acc;
          } else {
            angular_count = 0;
            clearInterval(left_stop_interval);
          }
          message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
          cmdVel.publish(
            new ROSLIB.Message({ angular: { x: 0, y: 0, z: angular_count } })
          );
        }, 25);
      } else if (event.keyCode == 68 || event.keyCode == 39) {
        /* d */ /* right */
        right_stop_interval = setInterval(function () {
          if (angular_count < 0) {
            angular_count += angular_dec;
          } else {
            angular_count = 0;
            clearInterval(right_stop_interval);
          }
          message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
          cmdVel.publish(
            new ROSLIB.Message({ angular: { x: 0, y: 0, z: angular_count } })
          );
        }, 25);
      }
    }
  });

  forward_movement_timer = null;
  backward_movement_timer = null;
  left_movement_timer = null;
  right_movement_timer = null;

  stop_forward_movement_timer = null;
  stop_backward_movement_timer = null;
  stop_left_movement_timer = null;
  stop_right_movement_timer = null;

  // -------------------acceleration from buttons------------------

  function forward_movement() {
    console.log("clearInterval stop_forward_movement_timer");
    if (stop_forward_movement_timer) clearInterval(stop_forward_movement_timer);
    console.log("forward_movement_timer");
    forward_movement_timer = setInterval(forward_acceleration, 50);
  }

  function backward_movement() {
    console.log("clearInterval stop_backward_movement_timer");
    if (stop_backward_movement_timer)
      clearInterval(stop_backward_movement_timer);
    console.log("backward_movement_timer");
    backward_movement_timer = setInterval(backward_acceleration, 50);
  }

  function right_movement() {
    console.log("clearInterval stop_left_movement_timer");
    if (stop_right_movement_timer) clearInterval(stop_right_movement_timer);
    console.log("right_movement_timer");
    right_movement_timer = setInterval(right_acceleration, 50);
  }

  function left_movement() {
    console.log("clearInterval stop_left_movement_timer");
    if (stop_left_movement_timer) clearInterval(stop_left_movement_timer);
    console.log("left_movement_timer");
    left_movement_timer = setInterval(left_acceleration, 50);
  }

  function forward_acceleration() {
    console.log("cmd_vel_lin++");
    const value = linear_сount + linear_acc;
    if (value > linear_max) {
      linear_сount = linear_max;
    } else {
      linear_сount = value;
    }
    message_listener.publish(new ROSLIB.Message({ data: "Moving FORWARD" }));
    cmdVel.publish(
      new ROSLIB.Message({ linear: { x: linear_сount, y: 0, z: 0 } })
    );
  }

  function backward_acceleration() {
    console.log("cmd_vel_lin--");
    const value = linear_сount - linear_acc;
    if (value < -linear_max) {
      linear_сount = -linear_max;
    } else {
      linear_сount = value;
    }
    message_listener.publish(new ROSLIB.Message({ data: "Moving BACKWARD" }));
    cmdVel.publish(
      new ROSLIB.Message({ linear: { x: linear_сount, y: 0, z: 0 } })
    );
  }

  function right_acceleration() {
    console.log("cmd_vel_ang--");
    const value = angular_count - angular_acc;
    if (value < -angular_max) {
      angular_count = -angular_max;
    } else {
      angular_count = value;
    }
    message_listener.publish(new ROSLIB.Message({ data: "Moving RIGHT" }));
    cmdVel.publish(
      new ROSLIB.Message({ angular: { x: 0, y: 0, z: angular_count } })
    );
  }

  function left_acceleration() {
    console.log("cmd_vel_ang++");
    const value = angular_count + angular_acc;
    if (value > angular_max) {
      angular_count = angular_max;
    } else {
      angular_count = value;
    }
    message_listener.publish(new ROSLIB.Message({ data: "Moving LEFT" }));
    cmdVel.publish(
      new ROSLIB.Message({ angular: { x: 0, y: 0, z: angular_count } })
    );
  }

  // -------------------deceleration from buttons------------------
  function stop_forward_Movement() {
    console.log("clearInterval forward_movement_timer");
    if (forward_movement_timer) clearInterval(forward_movement_timer);
    console.log("stop_forward_movement_timer");
    stop_forward_movement_timer = setInterval(forward_Deceleration, 50);
  }

  function stop_backward_Movement() {
    console.log("clearInterval backward_movement_timer");
    if (backward_movement_timer) clearInterval(backward_movement_timer);
    console.log("stop_backward_movement_timer");
    stop_backward_movement_timer = setInterval(backward_Deceleration, 50);
  }
  function stop_right_Movement() {
    console.log("clearInterval right_movement_timer");
    if (right_movement_timer) clearInterval(right_movement_timer);
    console.log("stop_right_movement_timer");
    stop_right_movement_timer = setInterval(right_Deceleration, 50);
  }

  function stop_left_Movement() {
    console.log("clearInterval left_movement_timer");
    if (left_movement_timer) clearInterval(left_movement_timer);
    console.log("stop_left_movement_timer");
    stop_left_movement_timer = setInterval(left_Deceleration, 50);
  }

  function forward_Deceleration() {
    const value = linear_сount - linear_dec;
    if (value > 0) {
      linear_сount = value;
    } else {
      linear_сount = 0;
    }
    console.log("lin++");
    message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
    cmdVel.publish(
      new ROSLIB.Message({ linear: { x: linear_сount, y: 0, z: 0 } })
    );
    if (linear_сount === 0) {
      console.log("clearInterval stop_forward_movement_timer");
      if (stop_forward_movement_timer)
        clearInterval(stop_forward_movement_timer);
    }
  }

  function backward_Deceleration() {
    const value = linear_сount + linear_dec;
    if (value < 0) {
      linear_сount = value;
    } else {
      linear_сount = 0;
    }
    console.log("lin--");
    message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
    cmdVel.publish(
      new ROSLIB.Message({ linear: { x: linear_сount, y: 0, z: 0 } })
    );
    if (linear_сount === 0) {
      console.log("clearInterval stop_backward_movement_timer");
      if (stop_backward_movement_timer)
        clearInterval(stop_backward_movement_timer);
    }
  }

  function right_Deceleration() {
    const value = angular_count + angular_dec;
    if (value < 0) {
      angular_count = value;
    } else {
      angular_count = 0;
    }
    console.log("ang--");
    message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
    cmdVel.publish(
      new ROSLIB.Message({ angular: { x: 0, y: 0, z: angular_count } })
    );
    if (angular_count === 0) {
      console.log("clearInterval stop_right_movement_timer");
      if (stop_right_movement_timer) clearInterval(stop_right_movement_timer);
    }
  }

  function left_Deceleration() {
    const value = angular_count - angular_dec;
    if (value > 0) {
      angular_count = value;
    } else {
      angular_count = 0;
    }
    console.log("ang++");
    message_listener.publish(new ROSLIB.Message({ data: "STOP" }));
    cmdVel.publish(
      new ROSLIB.Message({ angular: { x: 0, y: 0, z: angular_count } })
    );
    if (angular_count === 0) {
      console.log("clearInterval stop_left_movement_timer");
      if (stop_left_movement_timer) clearInterval(stop_left_movement_timer);
    }
  }

  forwardButton.onmousedown = forward_movement;
  backwardButton.onmousedown = backward_movement;
  rightButton.onmousedown = right_movement;
  leftButton.onmousedown = left_movement;

  forwardButton.onmouseup = stop_forward_Movement;
  backwardButton.onmouseup = stop_backward_Movement;
  leftButton.onmouseup = stop_left_Movement;
  rightButton.onmouseup = stop_right_Movement;

  zoomInButton.onclick = function (event) {
    event.preventDefault();
    zoomInMap(ros, viewer);
    const text = document.getElementById("statistics__desc").innerHTML;
    document.getElementById("statistics__desc__previous").innerHTML = text;
    document.getElementById(
      "statistics__desc"
    ).innerHTML = `[INFO]: Map zoomed in`;
    console.log("zoomInButton");
  };

  zoomOutButton.onclick = function (event) {
    event.preventDefault();
    zoomOutMap(ros, viewer);
    const text = document.getElementById("statistics__desc").innerHTML;
    document.getElementById("statistics__desc__previous").innerHTML = text;
    document.getElementById(
      "statistics__desc"
    ).innerHTML = `[INFO]: Map zoomed out`;
    console.log("zoomOutButton");
  };

  followButton.onclick = function () {
    const text = document.getElementById("statistics__desc").innerHTML;
    document.getElementById("statistics__desc__previous").innerHTML = text;
    document.getElementById(
      "statistics__desc"
    ).innerHTML = `[INFO]: Start following the route`;
    uiOperation.publish(followMessage);
    console.log("followButton");
  };

  clearButton.onclick = function () {
    uiOperation.publish(clearMessage);
    may_set_point.publish(new ROSLIB.Message({ data: true }));
    console.log("clearButton");
  };

  saveWpButton.onclick = function () {
    uiOperation.publish(saveWPMessage);
    may_set_point.publish(new ROSLIB.Message({ data: false }));
    console.log("saveWpButton");
  };

  buildMapButton.onclick = function () {
    may_set_point.publish(new ROSLIB.Message({ data: false }));
    uiOperation.publish(clearMessage);
    uiOperation.publish(buildMapMessage);
    console.log("buildMapButton");
  };

  homeButton.onclick = function () {
    uiOperation.publish(homeMessage);
    console.log("homeButton");
  };

  brakeButton.onclick = function () {
    uiOperation.publish(brakeMessage);
    console.log("brakeButton");
  };

  nextButton.onclick = function () {
    uiOperation.publish(nextMessage);
    console.log("nextButton");
  };

  previousButton.onclick = function () {
    uiOperation.publish(previousMessage);
    console.log("previousButton");
  };

  saveMapButton.onclick = function () {
    uiOperation.publish(saveMapMessage);
    console.log("saveMapButton");
  };

  message_listener.subscribe(function (message) {
    const text = document.getElementById("statistics__desc").innerHTML;
    if (text !== `[INFO]: ${message.data}`) {
      document.getElementById("statistics__desc__previous").innerHTML = text;
      document.getElementById(
        "statistics__desc"
      ).innerHTML = `[INFO]: ${message.data}`;
    }
  });

  velocity_listener.subscribe(function (message) {
    let lin_speed = message.linear.x.toFixed(2);
    let ang_speed = message.angular.z.toFixed(2);
    document.getElementById(
      "velocity__desc__previous"
    ).innerHTML = `Linear speed: ${lin_speed} m/s`;
    document.getElementById(
      "velocity__desc"
    ).innerHTML = `Angular speed: ${ang_speed} r/s`;
  });
};
