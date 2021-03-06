/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 */

var NAV2D = NAV2D || {
  REVISION : '0.3.0'
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A OccupancyGridClientNav uses an OccupancyGridClient to create a map for use with a Navigator.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map meta data topic to listen to
 *   * image - the URL of the image to render
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 *   * viewer - the main viewer to render to
 */
NAV2D.ImageMapClientNav = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || '/map_metadata';
  var image = options.image;
  this.serverName = options.serverName || '/move_base';
  this.actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;
  this.withOrientation = options.withOrientation || false;

  this.navigator = null;

  // setup a client to get the map
  var client = new ROS2D.ImageMapClient({
    ros : this.ros,
    rootObject : this.rootObject,
    topic : topic,
    image : image
  });
  client.on('change', function() {
    that.navigator = new NAV2D.Navigator({
      ros : that.ros,
      serverName : that.serverName,
      actionName : that.actionName,
      rootObject : that.rootObject,
      withOrientation : that.withOrientation
    });

    // scale the viewer to fit the map
    that.viewer.scaleToDimensions(client.currentImage.width, client.currentImage.height);
    that.viewer.shift(client.currentImage.pose.position.x, client.currentImage.pose.position.y);
  });
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. If
 * withOrientation is set to true, the user can also specify the orientation of
 * the robot by clicking at the goal position and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * serverName (optional) - the action server name to use for navigation (default '/move_base')
 *   * actionName (optional) - the navigation action name (default: 'move_base_msgs/MoveBaseAction')
 *   * poseTopicName (optional) - the topic name to get robot position (default '/robot_pose')
 *   * poseMessageType (optional) - poseTopicName's message type (default 'geometry_msgs/Pose')
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 *   * index (optional) - useful if multiple robots are present
 */
NAV2D.Navigator = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var serverName = options.serverName || '/move_base';
  var actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  var poseTopicName = options.poseTopicName || '/robot_pose';
  var poseMessageType = options.poseMessageType || 'geometry_msgs/Pose';
  var withOrientation = options.withOrientation || false;
  var index = options.index;
  this.rootObject = options.rootObject || new createjs.Container();
  this.nav = options.nav;

  // setup the actionlib client
  var securityActionClient = new ROSLIB.ActionClient({
    ros : ros,
    actionName : 'multi_robots_security_system/RobotPatrolAction',
    serverName : '/robot' + (index+1) + '/web'
  });

  /**
   * Send a goal to the navigation stack with the given pose.
   *
   * @param poses - an array of poses (if mode is investigate, should be an array with a single pose)
   * @param action - can be 'investigate' or 'patrol'
   */
  this.sendGoal = function(poses, action) {

    var goal = new ROSLIB.Goal({
      actionClient : securityActionClient,
      goalMessage : {
          patrol_poses : {
            header : {
              frame_id : '/map'
            },
            poses : poses
          }
        }
    });

    var goalMarker = null;
    // create a marker if action is 'investigate
    if(action === 'investigate') {
      window.app.changeRobotStatus(index, 'moving...');
      goalMarker = that.nav.createMarker(poses[0]);
      that.rootObject.addChild(goalMarker);
    } 

    goal.send();

    // handle goal result
    goal.on('result', function(result) {
      goal.status.status === 3 && window.app.changeRobotStatus(index, result.status);
      goalMarker && that.rootObject.removeChild(goalMarker);
    });
  }

  /**
   * Begins patrol on the robot 
   *
   * @param coordinates - a list of coordinates to patrol
   */
  this.startPatrol = function(coordinates) {
    if(coordinates.length === 0) return;

    // create pose messages for each coordinate
    var poses = [];

    for(var i=0; i<coordinates.length; i++) {
      poses.push(that.nav.createPoseMessage(coordinates[i].x, coordinates[i].y, coordinates[i].theta))
    }

    // send goal to robot
    that.sendGoal(poses, 'patrol');
  }

  this.stopAllRobotActions = function() {
    that.sendGoal([]);
  }

  var fillColor = createjs.Graphics.getRGB(255, 128, 0, 0.66);
  if(index === 1) {
    fillColor = createjs.Graphics.getRGB(128, 255, 0, 0.66);
  }

  // marker for the robot
  var robotMarker = new ROS2D.NavigationArrow({
    size : 25,
    strokeSize : 1,
    fillColor : fillColor,
    pulse : true
  });

  // wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = false;

  // setup a listener for the robot pose
  var poseListener = new ROSLIB.Topic({
    ros : ros,
    name : poseTopicName,
    messageType : poseMessageType,
    throttle_rate : 100
  });

  poseListener.subscribe(function(pose) {
    // workaround to handle different types of poseMessageType
    var poseData = null;
    if(pose.position) {
      poseData = pose;
    } else if(pose.pose && pose.pose.position) {
      poseData = pose.pose;
    } else if(pose.pose && pose.pose.pose && pose.pose.pose.position) {
      poseData = pose.pose.pose;
    }
    // update the robots position on the map
    robotMarker.x = poseData.position.x;
    robotMarker.y = -poseData.position.y;
    if (!initScaleSet) {
      robotMarker.scaleX = 0.5 / that.nav.stage.scaleX;
      robotMarker.scaleY = 0.5 / that.nav.stage.scaleY;
      initScaleSet = true;
    }

    // change the angle
    robotMarker.rotation = that.nav.stage.rosQuaternionToGlobalTheta(poseData.orientation);

    robotMarker.visible = true;
  });

  if (withOrientation === false){
    // setup a double click listener (no orientation)
    this.rootObject.addEventListener('dblclick', function(event) {
      var pose = that.nav.createPoseMessage(event.stageX, event.stageY);
      // send the goal if this robot is the one selected
      // without this check, every robot would receive the goal
      if(index === window.app.selectedRobotIndex) {
        that.sendGoal([pose], 'investigate');
      }
    });
  } else { // withOrientation === true
    // setup a click-and-point listener (with orientation)
    var position = null;
    var positionVec3 = null;
    var thetaRadians = 0;
    var thetaDegrees = 0;
    var orientationMarker = null;
    var mouseDown = false;
    var xDelta = 0;
    var yDelta = 0;

    var mouseEventHandler = function(event, mouseState) {

      if (mouseState === 'down'){
        // get position when mouse button is pressed down
        position = that.nav.stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new ROSLIB.Vector3(position);
        mouseDown = true;
      }
      else if (mouseState === 'move'){
        // remove obsolete orientation marker
        that.rootObject.removeChild(orientationMarker);
        
        if ( mouseDown === true) {
          // if mouse button is held down:
          // - get current mouse position
          // - calulate direction between stored <position> and current position
          // - place orientation marker
          var currentPos = that.nav.stage.globalToRos(event.stageX, event.stageY);
          var currentPosVec3 = new ROSLIB.Vector3(currentPos);

          orientationMarker = new ROS2D.NavigationArrow({
            size : 25,
            strokeSize : 1,
            fillColor : createjs.Graphics.getRGB(0, 255, 0, 0.66),
            pulse : false
          });

          xDelta =  currentPosVec3.x - positionVec3.x;
          yDelta =  currentPosVec3.y - positionVec3.y;
          
          thetaRadians  = Math.atan2(xDelta,yDelta);

          thetaDegrees = thetaRadians * (180.0 / Math.PI);
          
          if (thetaDegrees >= 0 && thetaDegrees <= 180) {
            thetaDegrees += 270;
          } else {
            thetaDegrees -= 90;
          }

          orientationMarker.x =  positionVec3.x;
          orientationMarker.y = -positionVec3.y;
          orientationMarker.rotation = thetaDegrees;
          orientationMarker.scaleX = 0.5 / that.nav.stage.scaleX;
          orientationMarker.scaleY = 0.5 / that.nav.stage.scaleY;
          
          that.rootObject.addChild(orientationMarker);
        }
      } else if (mouseDown) { // mouseState === 'up'
        // if mouse button is released
        // - get current mouse position (goalPos)
        // - calulate direction between stored <position> and goal position
        // - set pose with orientation
        // - send goal
        mouseDown = false;

        // calculate orientation
        var goalPos = that.nav.stage.globalToRos(event.stageX, event.stageY);
        var goalPosVec3 = new ROSLIB.Vector3(goalPos);
        xDelta =  goalPosVec3.x - positionVec3.x;
        yDelta =  goalPosVec3.y - positionVec3.y;
        thetaRadians  = Math.atan2(xDelta,yDelta);
        var orientationParams = that.nav.getOrientationParams(thetaRadians);
        var orientation = new ROSLIB.Quaternion({x:0, y:0, z: orientationParams.qz, w: orientationParams.qw});
        
        // send goal
        var pose = new ROSLIB.Pose({
          position :    positionVec3,
          orientation : orientation
        });
        // send the goal if this robot is the one selected
        // without this check, every robot would receive the goal
        if(index === window.app.selectedRobotIndex) {
          that.sendGoal([pose], 'investigate');
        } 
      }
    };

    this.rootObject.addEventListener('stagemousedown', function(event) {
      mouseEventHandler(event,'down');
    });

    this.rootObject.addEventListener('stagemousemove', function(event) {
      mouseEventHandler(event,'move');
    });

    this.rootObject.addEventListener('stagemouseup', function(event) {
      mouseEventHandler(event,'up');
    });
  }
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A OccupancyGridClientNav uses an OccupancyGridClient to create a map for use with a Navigator.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * rootObject (optional) - the root object to add this marker to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * serverName (optional) - the action server name to use for navigation (default: '/move_base')
 *                             can be a string (single robot) or an array of strings (multiple robots)
 *   * poseTopicName (optional) - the topic name to get robot position (default '/robot_pose')
 *                                can be a string (single robot) or an array of strings (multiple robots)  
 *   * poseMessageType (optional) - poseTopicName's message type (default 'geometry_msgs/Pose')
 *   * actionName (optional) - the navigation action name (default: 'move_base_msgs/MoveBaseAction')
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 *   * viewer - the main viewer to render to
 */
NAV2D.OccupancyGridClientNav = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || '/map';
  var continuous = options.continuous;
  this.serverName = options.serverName || '/move_base';
  this.actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.poseTopicName = options.poseTopicName || '/robot_pose';
  this.poseMessageType = options.poseMessageType || 'geometry_msgs/Pose';
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;
  this.withOrientation = options.withOrientation || false;
  this.robotColors = options.robotColors
  this.robots = [];
  this.stage = null;

  // get a handle to the stage
  if (this.rootObject instanceof createjs.Stage) {
    this.stage = this.rootObject;
  } else {
    this.stage = this.rootObject.getStage();
  }

  /**
   * Get z and w orientation params of the quaternion
   *
   * @param theta - orientation of the robot in radians (optional)
   */
  this.getOrientationParams = function(theta) {
    if (theta >= 0 && theta <= Math.PI) {
      theta += (3 * Math.PI / 2);
    } else {
      theta -= (Math.PI/2);
    }
    var qz =  Math.sin(-theta/2.0);
    var qw =  Math.cos(-theta/2.0);
    return {qz: qz, qw: qw};
  }

  /**
   * Create a pose message to send to the robot
   *
   * @param x - position x of the map
   * @param y - position y of the map
   * @param theta - orientation of the robot in radians (optional)
   */
  
  this.createPoseMessage = function(x, y, theta) {
    // convert map coordinates to ROS coordinates
    var coords = that.stage.globalToRos(x, y);
    var config = {
      position : new ROSLIB.Vector3(coords)
    }
    if(theta) {
      var orientation = that.getOrientationParams(theta);
      config.orientation = new ROSLIB.Quaternion({x:0, y:0, z: orientation.qz, w: orientation.qw});
    }
    return new ROSLIB.Pose(config);
  }

     /**
   * Create a marker on the map.
   *
   * @param pose - the pose that the marker will be based upon
   * @param customConfig - optional config for the marker
   */
  this.createMarker = function(pose, customConfig) {
    var defaultConfig = {
      size : 15,
      strokeSize : 1,
      fillColor : createjs.Graphics.getRGB(255, 64, 128, 0.66),
      pulse : true
    }
    var config = customConfig || defaultConfig;
    var marker = new ROS2D.NavigationArrow(config);
    marker.x = pose.position.x;
    marker.y = -pose.position.y;
    marker.rotation = that.stage.rosQuaternionToGlobalTheta(pose.orientation);
    marker.scaleX = 0.5 / that.stage.scaleX;
    marker.scaleY = 0.5 / that.stage.scaleY;
    return marker;
  }

  // setup a client to get the map
  var client = new ROS2D.OccupancyGridClient({
    ros : this.ros,
    rootObject : this.rootObject,
    continuous : continuous,
    topic : topic
  });
  client.on('change', function() {
    // create one navigator for each robot
    if(typeof that.serverName === 'object' && that.serverName != null) {
      for(var i=0; i<that.serverName.length; i++) {
        that.robots.push({});
        that.robots[i].navigator = new NAV2D.Navigator({
          ros : that.ros,
          nav: that,
          serverName : that.serverName[i],
          actionName : that.actionName,
          rootObject : that.rootObject,
          withOrientation : that.withOrientation,
          poseTopicName: that.poseTopicName[i],
          poseMessageType: that.poseMessageType,
          index: i
        });
        that.robots[i].color = that.robotColors[i];
        that.robots[i].status = 'available';
      }
    }
    // scale the viewer to fit the map
    that.viewer.scaleToDimensions(client.currentGrid.width, client.currentGrid.height);
    that.viewer.shift(client.currentGrid.pose.position.x, client.currentGrid.pose.position.y);
  });

  return that;
};