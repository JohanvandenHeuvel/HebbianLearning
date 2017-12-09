/* Didabots - robot and GUI script.
 *
 * Copyright 2016 Harmen de Weerd
 * Copyright 2017 Johannes Keyser, James Cooke
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



//Hardcoding this shit
Boxes = []
averageHeapSize = [];
percentageInAHeap = [];
amountOfHeaps = [];
groupDistribution = [];
boxesMoved = [];
lastPositions = [];
//fs = null;

// Description of robot(s), and attached sensor(s) used by InstantiateRobot()
const sensorlength = 30;

RobotInfo = [
  {body: null,  // for MatterJS body, added by InstantiateRobot()
   color: "red",  // color of the robot marker
   init: {x: 50, y: 50, angle: 0},  // initial position and orientation
   sensors: [  // define an array of sensors on the robot
       // define 45 degrees right sensor
       {sense: senseDouble,  // function handle, determines type of sensor
           minVal: 0,  // minimum detectable distance, in pixels
           maxVal: sensorlength,  // maximum detectable distance, in pixels
           attachAngle: Math.PI/4,  // where the sensor is mounted on robot body
           lookAngle: 0,  // direction the sensor is looking (relative to center-out)
           id: 'distR3',  // a unique, arbitrary ID of the sensor, for printing/debugging
           parent: null,  // robot object the sensor is attached to, added by InstantiateRobot
           value: null  // sensor value, i.e. distance in pixels; updated by sense() function
       },
       // def 45 degrees left sensor
       {sense: senseDouble,
           minVal: 0,
           maxVal: sensorlength,
           attachAngle: -Math.PI/4,
           lookAngle: 0,
           id: 'distR1',
           parent: null,
           value: null}
   ]
  }
];

simInfo = {
  maxSteps: 20000,  // maximal number of simulation steps to run
  airDrag: 0.1,  // "air" friction of enviroment; 0 is vacuum, 0.9 is molasses
  boxFric: 0.005,
  boxMass: 1,  // mass of boxes
  boxSize: 18,  // size of the boxes, in pixels
  robotSize: 14,//2*7,  // robot radius, in pixels
  robotMass: 0.4, // robot mass (a.u)
  gravity: 0,  // constant acceleration in Y-direction
  bayRobot: null,  // currently selected robot
  baySensor: null,  // currently selected sensor
  bayScale: 3,  // scale within 2nd, inset canvas showing robot in it's "bay"
  doContinue: true,  // whether to continue simulation, set in HTML
  debugSensors: true,  // plot sensor rays and mark detected objects
  debugMouse: true,  // allow dragging any object with the mouse
  engine: null,  // MatterJS 2D physics engine
  world: null,  // world object (composite of all objects in MatterJS engine)
  runner: null,  // object for running MatterJS engine
  height: null,  // set in HTML file; height of arena (world canvas), in pixels
  width: null,  // set in HTML file; width of arena (world canvas), in pixels
  curSteps: 0  // increased by simStep()
};

robots = new Array();
sensors = new Array();

function init() {  // called once when loading HTML file
  const robotBay = document.getElementById("bayDidabot"),
        arena = document.getElementById("arenaDidabot"),
        height = arena.height,
        width = arena.width;
  simInfo.height = height;
  simInfo.width = width;

  /* Create a MatterJS engine and world. */
  simInfo.engine = Matter.Engine.create();
  simInfo.world = simInfo.engine.world;
  simInfo.world.gravity.y = simInfo.gravity;
  simInfo.engine.timing.timeScale = 1;

  /* Create walls and boxes, and add them to the world. */
  // note that "roles" are custom properties for rendering (not from MatterJS)
  function getWall(x, y, width, height) {
    return Matter.Bodies.rectangle(x, y, width, height,
                                   {isStatic: true, role: 'wall'});
  };
  const wall_lo = getWall(width/2, height-5, width-5, 5),
        wall_hi = getWall(width/2, 5, width-5, 5),
        wall_le = getWall(5, height/2, 5, height-15),
        wall_ri = getWall(width-5, height/2, 5, height-15);
  Matter.World.add(simInfo.world, [wall_lo, wall_hi, wall_le, wall_ri]);

  /* Add a bunch of boxes in a neat grid. */
  function getBox(x, y) {
      //change the 3 to 4 or alot(circle)
    return Matter.Bodies.polygon(350*Math.random() + 25, 350*Math.random() + 25, 4, simInfo.boxSize,
                                   {frictionAir: simInfo.airDrag,
                                    friction: simInfo.boxFric,
                                    mass: simInfo.boxMass,
                                    role: 'box'});
  };
  const startX = 100, startY = 100,
        nBoxX = 5, nBoxY = 5,
        gapX = 40, gapY = 30,
        stack = Matter.Composites.stack(startX, startY,
                                        nBoxX, nBoxY,
                                        gapX, gapY, getBox);

  boxes = stack.bodies;
  Matter.World.add(simInfo.world, stack);

  /* Add debugging mouse control for dragging objects. */
  if (simInfo.debugMouse){
    const mouseConstraint = Matter.MouseConstraint.create(simInfo.engine,
                              {mouse: Matter.Mouse.create(arena),
                               // spring stiffness mouse ~ object
                               constraint: {stiffness: 0.5}});
    Matter.World.add(simInfo.world, mouseConstraint);
  }
  // Add the tracker functions from mouse.js
  addMouseTracker(arena);
  addMouseTracker(robotBay);

  /* Running the MatterJS physics engine (without120 rendering). */
  simInfo.runner = Matter.Runner.create({delta:1000/400,
                                            fps: 60,  // TODO: why weird effects?
                                         isFixed: false});
  Matter.Runner.start(simInfo.runner, simInfo.engine);
  // register function simStep() as callback to MatterJS's engine events
  Matter.Events.on(simInfo.engine, 'tick', simStep);

  /* Create robot(s). */
//<<<<<<< Updated upstream
  setRobotNumber(1);  // requires defined simInfo.world
//=======
  //setRobotNumber(1);  // requires defined simInfo.world
//>>>>>>> Stashed changes
  loadBay(robots[0]);

};

function rotate(robot, torque = 0) {
  /* Apply a torque to the robot to rotate it.
   *
   * Parameters
   *   torque - rotational force to apply to the body.
   */
  robot.body.torque = torque;
 };

function drive(robot, force=0) {
  /* Apply a force to the robot to move it.
   *
   * Parameters
   *   force - force to apply to the body.
   */
  const orientation = robot.body.angle,
        force_vec = Matter.Vector.create(force, 0),
        move_vec = Matter.Vector.rotate(force_vec, orientation);
  Matter.Body.applyForce(robot.body, robot.body.position , move_vec);
};

function senseDouble() {
    /* SENSES BOTH DISTANCE AND TOUCH
     * Distance sensor simulation based on ray casting. Called from sensor
     * object, returns nothing, updates a new reading into this.value.
     *
     * Idea: Cast a ray with a certain length from the sensor, and check
     *       via collision detection if objects intersect with the ray.
     *       To determine distance, run a Binary search on ray length.
     * Note: Sensor ray needs to ignore robot (parts), or start outside of it.
     *       The latter is easy with the current circular shape of the robots.
     * Note: Order of tests are optimized by starting with max ray length, and
     *       then only testing the maximal number of initially resulting objects.
     * Note: The sensor's "ray" could have any other (convex) shape;
     *       currently it's just a very thin rectangle.
     */

    const context = document.getElementById('arenaDidabot').getContext('2d');
    var bodies = Matter.Composite.allBodies(simInfo.engine.world);
    bodies = bodies.filter(function(x){return !(x.label==='point')});
    var touchTreshold = 5;

    const robotAngle = this.parent.body.angle,
        attachAngle = this.attachAngle,
        rayAngle = robotAngle + attachAngle + this.lookAngle;

    const rPos = this.parent.body.position,
        rSize = simInfo.robotSize,
        startPoint = {x: rPos.x + (rSize+1) * Math.cos(robotAngle + attachAngle),
            y: rPos.y + (rSize+1) * Math.sin(robotAngle + attachAngle)};

    function getEndpoint(rayLength) {
        return {x: startPoint.x + rayLength * Math.cos(rayAngle),
            y: startPoint.y + rayLength * Math.sin(rayAngle)};
    };

    function sensorRay(bodies, rayLength) {
        // Cast ray of supplied length and return the bodies that collide with it.
        const rayWidth = 1e-100,
            endPoint = getEndpoint(rayLength);
        rayX = (endPoint.x + startPoint.x) / 2,
            rayY = (endPoint.y + startPoint.y) / 2,
            rayRect = Matter.Bodies.rectangle(rayX, rayY, rayLength, rayWidth,
                {isSensor: true, isStatic: true,
                    angle: rayAngle, role: 'sensor'});

        var collidedBodies = [];
        for (var bb = 0; bb < bodies.length; bb++) {
            var body = bodies[bb];
            // coarse check on body boundaries, to increase performance:
            if (Matter.Bounds.overlaps(body.bounds, rayRect.bounds)) {
                for (var pp = body.parts.length === 1 ? 0 : 1; pp < body.parts.length; pp++) {
                    var part = body.parts[pp];
                    // finer, more costly check on actual geometry:
                    if (Matter.Bounds.overlaps(part.bounds, rayRect.bounds)) {
                        const collision = Matter.SAT.collides(part, rayRect);
                        if (collision.collided) {
                            collidedBodies.push(body);
                            break;
                        }
                    }
                }
            }
        }
        return collidedBodies;
    };

    // call 1x with full length, and check all bodies in the world;
    // in subsequent calls, only check the bodies resulting here
    var rayLength = this.maxVal;
    bodies = sensorRay(bodies, rayLength);

    // if some collided, search for maximal ray length without collisions
    if (bodies.length > 0) {
        var lo = 0,
            hi = rayLength;
        while (lo < rayLength) {
            if (sensorRay(bodies, rayLength).length > 0) {
                hi = rayLength;
            }
            else {
                lo = rayLength;
            }
            rayLength = Math.floor(lo + (hi-lo)/2);
        }
    }
    // increase length to (barely) touch closest body (if any)
    rayLength += 1;
    bodies = sensorRay(bodies, rayLength);

    if (simInfo.debugSensors) {  // if invisible, check order of object drawing
        // draw the resulting ray
        endPoint = getEndpoint(rayLength);
        context.beginPath();
        context.moveTo(startPoint.x, startPoint.y);
        context.lineTo(endPoint.x, endPoint.y);
        context.strokeStyle = this.parent.info.color;
        context.lineWidth = 0.5;
        context.stroke();
        endPointTouch = getEndpoint(touchTreshold);
        context.beginPath();
        context.moveTo(startPoint.x, startPoint.y);
        context.lineTo(endPointTouch.x, endPointTouch.y);
        context.strokeStyle = this.parent.info.color;
        context.lineWidth = 4.0;
        context.stroke();
        // mark all objects's lines intersecting with the ray
        for (var bb = 0; bb < bodies.length; bb++) {
            var vertices = bodies[bb].vertices;
            context.moveTo(vertices[0].x, vertices[0].y);
            for (var vv = 1; vv < vertices.length; vv += 1) {
                context.lineTo(vertices[vv].x, vertices[vv].y);
            }
            context.closePath();
        }
        context.lineWidth = 0.5;
        context.stroke();

    }

    // indicate if the sensor exceeded its maximum length by returning infinity
    if (rayLength > this.maxVal) {
        rayLength = Infinity;
    }
    else {
        // apply mild noise on the sensor reading, and clamp between valid values
        function gaussNoise(sigma=1) {
            const x0 = 1.0 - Math.random();
            const x1 = 1.0 - Math.random();
            return sigma * Math.sqrt(-2 * Math.log(x0)) * Math.cos(2 * Math.PI * x1);
        };
        rayLength = Math.floor(rayLength + gaussNoise(3));
        rayLength = Matter.Common.clamp(rayLength, this.minVal, this.maxVal);
    }


     this.value = {dist: rayLength,touch: rayLength<touchTreshold};
};

function senseDistance() {
  /* Distance sensor simulation based on ray casting. Called from sensor
   * object, returns nothing, updates a new reading into this.value.
   *
   * Idea: Cast a ray with a certain length from the sensor, and check
   *       via collision detection if objects intersect with the ray.
   *       To determine distance, run a Binary search on ray length.
   * Note: Sensor ray needs to ignore robot (parts), or start outside of it.
   *       The latter is easy with the current circular shape of the robots.
   * Note: Order of tests are optimized by starting with max ray length, and
   *       then only testing the maximal number of initially resulting objects.
   * Note: The sensor's "ray" could have any other (convex) shape;
   *       currently it's just a very thin rectangle.
   */

  const context = document.getElementById('arenaDidabot').getContext('2d');
  var bodies = Matter.Composite.allBodies(simInfo.engine.world);
  bodies = bodies.filter(function(x){return !(x.label==='point')});


  const robotAngle = this.parent.body.angle,
        attachAngle = this.attachAngle,
        rayAngle = robotAngle + attachAngle + this.lookAngle;

  const rPos = this.parent.body.position,
        rSize = simInfo.robotSize,
        startPoint = {x: rPos.x + (rSize+1) * Math.cos(robotAngle + attachAngle),
                      y: rPos.y + (rSize+1) * Math.sin(robotAngle + attachAngle)};

  function getEndpoint(rayLength) {
      return {x: startPoint.x + rayLength * Math.cos(rayAngle),
          y: startPoint.y + rayLength * Math.sin(rayAngle)};
  };

  function sensorRay(bodies, rayLength) {
    // Cast ray of supplied length and return the bodies that collide with it.
    const rayWidth = 1e-100,
          endPoint = getEndpoint(rayLength);
    rayX = (endPoint.x + startPoint.x) / 2,
    rayY = (endPoint.y + startPoint.y) / 2,
    rayRect = Matter.Bodies.rectangle(rayX, rayY, rayLength, rayWidth,
                                      {isSensor: true, isStatic: true,
                                       angle: rayAngle, role: 'sensor'});

    var collidedBodies = [];
    for (var bb = 0; bb < bodies.length; bb++) {
      var body = bodies[bb];
      // coarse check on body boundaries, to increase performance:
      if (Matter.Bounds.overlaps(body.bounds, rayRect.bounds)) {
        for (var pp = body.parts.length === 1 ? 0 : 1; pp < body.parts.length; pp++) {
          var part = body.parts[pp];
          // finer, more costly check on actual geometry:
          if (Matter.Bounds.overlaps(part.bounds, rayRect.bounds)) {
            const collision = Matter.SAT.collides(part, rayRect);
            if (collision.collided) {
              collidedBodies.push(body);
              break;
            }
          }
        }
      }
    }
    return collidedBodies;
  };

  // call 1x with full length, and check all bodies in the world;
  // in subsequent calls, only check the bodies resulting here
  var rayLength = this.maxVal;
  bodies = sensorRay(bodies, rayLength);

  // if some collided, search for maximal ray length without collisions
  if (bodies.length > 0) {
    var lo = 0,
        hi = rayLength;
    while (lo < rayLength) {
      if (sensorRay(bodies, rayLength).length > 0) {
        hi = rayLength;
      }
      else {
        lo = rayLength;
      }
      rayLength = Math.floor(lo + (hi-lo)/2);
    }
  }
  // increase length to (barely) touch closest body (if any)
  rayLength += 1;
  bodies = sensorRay(bodies, rayLength);

  if (simInfo.debugSensors) {  // if invisible, check order of object drawing
    // draw the resulting ray
    endPoint = getEndpoint(rayLength);
    context.beginPath();
    context.moveTo(startPoint.x, startPoint.y);
    context.lineTo(endPoint.x, endPoint.y);
    context.strokeStyle = this.parent.info.color;
    context.lineWidth = 0.5;
    context.stroke();
    // mark all objects's lines intersecting with the ray
    for (var bb = 0; bb < bodies.length; bb++) {
      var vertices = bodies[bb].vertices;
      context.moveTo(vertices[0].x, vertices[0].y);
      for (var vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      context.closePath();
    }
    context.stroke();
  }

  // indicate if the sensor exceeded its maximum length by returning infinity
  if (rayLength > this.maxVal) {
    rayLength = Infinity;
  }
  else {
    // apply mild noise on the sensor reading, and clamp between valid values
    function gaussNoise(sigma=1) {
      const x0 = 1.0 - Math.random();
      const x1 = 1.0 - Math.random();
      return sigma * Math.sqrt(-2 * Math.log(x0)) * Math.cos(2 * Math.PI * x1);
    };
    rayLength = Math.floor(rayLength + gaussNoise(3));
    rayLength = Matter.Common.clamp(rayLength, this.minVal, this.maxVal);
  }

  this.value = rayLength;
};


function dragSensor(sensor, event) {
  const robotBay = document.getElementById('bayDidabot'),
        bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        rSize = simInfo.robotSize,
        bScale = simInfo.bayScale,
        sSize = sensor.getWidth(),
        mAngle = Math.atan2(  event.mouse.x - bCenter.x,
                            -(event.mouse.y - bCenter.y));
  sensor.info.attachAngle = mAngle;
  sensor.x = bCenter.x - sSize - bScale * rSize * Math.sin(-mAngle);
  sensor.y = bCenter.y - sSize - bScale * rSize * Math.cos( mAngle);
  repaintBay();
}

function loadSensor(sensor, event) {
  loadSensorInfo(sensor.sensor);
}

function loadSensorInfo(sensorInfo) {
  simInfo.baySensor = sensorInfo;
}

function loadBay(robot) {
  simInfo.bayRobot = robot;
  sensors = new Array();
  const robotBay = document.getElementById("bayDidabot");
  const bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        rSize = simInfo.robotSize,
        bScale = simInfo.bayScale;

  for (var ss = 0; ss < robot.info.sensors.length; ++ss) {
    const curSensor = robot.sensors[ss],
          attachAngle = curSensor.attachAngle;
    // put current sensor into global variable, make mouse-interactive
    sensors[ss] = makeInteractiveElement(new SensorGraphics(curSensor),
                                         document.getElementById("bayDidabot"));
    const sSize = sensors[ss].getWidth();
    sensors[ss].x = bCenter.x - sSize - bScale * rSize * Math.sin(-attachAngle);
    sensors[ss].y = bCenter.y - sSize - bScale * rSize * Math.cos( attachAngle);
    sensors[ss].onDragging = dragSensor;
    sensors[ss].onDrag = loadSensor;
  }
  repaintBay();
}

function SensorGraphics(sensorInfo) {
  this.info = sensorInfo;
  this.plotSensor = plotSensor;
  // add functions getWidth/getHeight for graphics.js & mouse.js,
  // to enable dragging the sensor in the robot bay
  this.getWidth = function() { return 6; };
  this.getHeight = function() { return 6; };
}

function InstantiateRobot(robotInfo) {
  // create robot's main physical body (simulated with MatterJS engine)
  const nSides = 20,
        circle = Matter.Bodies.circle;
  this.body = circle(robotInfo.init.x, robotInfo.init.y, simInfo.robotSize,
                     {frictionAir: simInfo.airDrag,
                       mass: simInfo.robotMass,
                       role: 'robot'}, nSides);
  Matter.World.add(simInfo.world, this.body);
  Matter.Body.setAngle(this.body, robotInfo.init.angle);

  // instantiate its sensors
  this.sensors = robotInfo.sensors;
  for (var ss = 0; ss < this.sensors.length; ++ss) {
    this.sensors[ss].parent = this;
  }

  // attach its helper functions
  this.rotate = rotate;
  this.drive = drive;
  this.info = robotInfo;
  this.plotRobot = plotRobot;

  // add functions getWidth/getHeight for graphics.js & mouse.js,
  // to enable selection by clicking the robot in the arena
  this.getWidth = function() { return 2 * simInfo.robotSize; };
  this.getHeight = function() { return 2 * simInfo.robotSize; };
}

function robotUpdateSensors(robot) {
  // update all sensors of robot; puts new values into sensor.value
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    robot.sensors[ss].sense();
  }
};

function getSensorValById(robot, id) {
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    if (robot.sensors[ss].id == id) {
      return robot.sensors[ss].value;
    }
  }
  return undefined;  // if not returned yet, id doesn't exist
};

function robotMove(robot) {
// This function is called each timestep and should be used to move the robots
  robotUpdateSensors(robot)
  rightSens = robot.sensors[0];
  leftSens = robot.sensors[1];

  leftDist = leftSens.value.dist == Infinity ? leftSens.maxVal : leftSens.value.dist;
  rightDist = rightSens.value.dist == Infinity ? rightSens.maxVal :  rightSens.value.dist;
  leftTouch = leftSens.value.touch
  rightTouch = rightSens.value.touch

  const angle = 0.005;
  const forward = 0.0004;

  leftTotal = (leftDist + 0.3 * Math.random() - 0.3 * Math.random())
  rightTotal = (rightDist + 0.3 * Math.random() - 0.3 * Math.random())

  leftAngle = leftTotal * angle;
  rightAngle = rightTotal * angle;

  direction = 0 - leftAngle + rightAngle; //rotates faster when far away

  drive(robot, forward);
  rotate(robot, direction);
};

function plotSensor(context, x = this.x, y = this.y) {
  context.beginPath();
  context.arc(x + this.getWidth()/2,
              y + this.getHeight()/2,
              this.getWidth()/2, 0, 2*Math.PI);
  context.closePath();
  context.fillStyle = 'black';
  context.strokeStyle = 'black';
  context.fill();
  context.stroke();
}

function plotRobot(context,
                     xTopLeft = this.body.position.x,
                     yTopLeft = this.body.position.y) {
  var x, y, scale, angle, i, half, full,
      rSize = simInfo.robotSize;

  if (context.canvas.id == "bayDidabot") {
    scale = simInfo.bayScale;
    half = Math.floor(rSize/2*scale);
    full = half * 2;
    x = xTopLeft + full;
    y = yTopLeft + full;
    angle = -Math.PI / 2;
  } else {
    scale = 1;
    half = Math.floor(rSize/2*scale);
    full = half * 2;
    x = xTopLeft;
    y = yTopLeft;
    angle = this.body.angle;
  }
  context.save();
  context.translate(x, y);
  context.rotate(angle);

  // Plot wheels as rectangles.
  context.strokeStyle = "black";
  if (context.canvas.id == "bayDidabot") {
    context.fillStyle = "grey";
    context.fillRect(-half, -full, full, full);
    context.fillRect(-half, 0, full, full);
  } else {
    context.fillStyle = "grey";
    context.fillRect(-half, -full, full, 2*full);
  }
  context.strokeRect(-half, -full, full, 2*full);

  // Plot circular base object.
  if (context.canvas.id == "bayDidabot") {
    context.beginPath();
    context.arc(0, 0, full, 0, 2*Math.PI);
    context.closePath();
    context.fillStyle = "lightgrey";
    context.fill();
    context.stroke();
  }
  else { // context.canvas.id == "arenaDidabot"
    // draw into world canvas without transformations,
    // because MatterJS thinks in world coords...
    context.restore();
    context.beginPath();
    var vertices = this.body.vertices;
    context.moveTo(vertices[0].x, vertices[0].y);
    for (var vv = 1; vv < vertices.length; vv += 1) {
      context.lineTo(vertices[vv].x, vertices[vv].y);
    }
    context.closePath();
    context.fillStyle = 'lightgrey';
    context.stroke();
    context.fill();
    // to draw the rest, rotate & translate again
    context.save();
    context.translate(x, y);
    context.rotate(angle);
  }

  // Plot a marker to distinguish robots and their orientation.
  context.beginPath();
  context.arc(0, 0, full * .4, -Math.PI/4, Math.PI/4);
  context.lineTo(full * Math.cos(Math.PI/4) * .8,
             full * Math.sin(Math.PI/4) * .8);
  context.arc(0, 0, full * .8, Math.PI/4, -Math.PI/4, true);
  context.closePath();
  context.fillStyle = this.info.color;
  context.fill();
  context.stroke();

  // Plot sensor positions into world canvas.
  if (context.canvas.id == "arenaDidabot") {
    for (ss = 0; ss < this.info.sensors.length; ++ss) {
      context.beginPath();
      context.arc(full * Math.cos(this.info.sensors[ss].attachAngle),
                  full * Math.sin(this.info.sensors[ss].attachAngle),
                  scale, 0, 2*Math.PI);
      context.closePath();
      context.fillStyle = 'black';
      context.strokeStyle = 'black';
      context.fill();
      context.stroke();
    }
  }
  context.restore();
}

function simStep() {
  // advance simulation by one step (except MatterJS engine's physics)
  if (simInfo.curSteps < simInfo.maxSteps) {
    repaintBay();
    drawBoard();
    for (var rr = 0; rr < robots.length; ++rr) {
      robotUpdateSensors(robots[rr]);
      robotMove(robots[rr]);
      // To enable selection by clicking (via mouse.js/graphics.js),
      // the position on the canvas needs to be defined in (x, y):
      const rSize = simInfo.robotSize;
      robots[rr].x = robots[rr].body.position.x - rSize;
      robots[rr].y = robots[rr].body.position.y - rSize;
    }
   // count and display number of steps
    simInfo.curSteps += 1;
    document.getElementById("SimStepLabel").innerHTML =
      padnumber(simInfo.curSteps, 5) +
      ' of ' +
      padnumber(simInfo.maxSteps, 5);
  }
  else {
    exportExcel();
    toggleSimulation();
  }

    log = function() {
        var context = "My Descriptive Logger Prefix:";
        return Function.prototype.bind.call(console.log, console, context);
    }();

  if (simInfo.curSteps%60 == 0){
    updateStatistics();
  }

    /*if (simInfo.curSteps > 19000){
    /*Matter.World.add(simInfo.world, [Matter.Bodies.rectangleCollisionless(robots[0].x+simInfo.robotSize/2, robots[0].y+simInfo.robotSize/2, 1, 1,//x, y, simInfo.boxSize, simInfo.boxSize,
        {frictionAir: simInfo.airDrag,
            friction: simInfo.boxFric,
            mass: simInfo.boxMass,
            role: 'point'})])};*/
}

function updateStatistics() {
    positions = boxes.map(function(x){return x.position});
    if (lastPositions.length == 0){
        lastPositions = positions.map(a => Object.assign({}, a));
	}

    positionsNotAtEdge = positions.filter(function(pos)
    {return (pos.x > 1.3*simInfo.boxSize + 5)&&
    (pos.x < simInfo.width - (1.3*simInfo.boxSize + 5))&&
        (pos.y > 1.3*simInfo.boxSize + 5)&&
        (pos.y < simInfo.height - (1.3*simInfo.boxSize + 5))});

        //console.log(positions.length - positionsNotAtEdge.length)
    groups = calculateGroups(positionsNotAtEdge);
    amountOfBoxesMoved = 0;
    tresHoldMoveM = 0.01
    for (var i = 0; i < positions.length; i++){
        if (Math.abs(lastPositions[i].x - positions[i].x) > tresHoldMoveM || Math.abs(lastPositions[i].y - positions[i].y) > tresHoldMoveM){
            amountOfBoxesMoved++;
        }
    }
    boxesMoved.push(amountOfBoxesMoved);
    heaps = groups.filter(function(x){return x.length>2});

    percentageInAHeap.push(flatten(heaps).length / positions.length);
    amountOfHeaps.push(heaps.length);
    averageHeapSize.push(averLength(heaps));

    groupSizes = groups.map(function(x){return x.length});
    distribution = []
    for (var i = 1; i <= positions.length; i++){
        distribution.push((groupSizes.filter(function(x){return x == i}).length*i) / positions.length)
    }
    distribution[0] += (positions.length - positionsNotAtEdge.length)/ positions.length
    groupDistribution.push(distribution)
    lastPositions = positions.map(a => Object.assign({}, a));
}

function drawBoard() {
  var context = document.getElementById('arenaDidabot').getContext('2d');
  context.fillStyle = "#444444";
  context.fillRect(0, 0, simInfo.width, simInfo.height);

  // draw objects within world
  const Composite = Matter.Composite,
        bodies = Composite.allBodies(simInfo.world);
  context.beginPath();
  for (var bb = 0; bb < bodies.length; bb += 1) {
    if (bodies[bb].role == 'robot') {
      // don't draw robot's circles; they're drawn below
      continue;
    }
    else {
      var vertices = bodies[bb].vertices;
      context.moveTo(vertices[0].x, vertices[0].y);
      for (var vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      context.closePath();
    }
  }
  context.lineWidth = 1;
  context.strokeStyle = '#999';
  context.fillStyle = '#5559';
  context.fill();
  context.stroke();

  // draw all robots
  for (var rr = 0; rr < robots.length; ++rr) {
    robots[rr].plotRobot(context);
  }
}

function repaintBay() {
  // update inset canvas showing information about selected robot
  const robotBay = document.getElementById('bayDidabot'),
        context = robotBay.getContext('2d');
  context.clearRect(0, 0, robotBay.width, robotBay.height);
  simInfo.bayRobot.plotRobot(context, 10, 10);
  for (var ss = 0; ss < sensors.length; ss++) {
    sensors[ss].plotSensor(context);
  }

  // print sensor values of selected robot next to canvas
  if (!(simInfo.curSteps % 5)) {  // update slow enough to read
    var sensorString = '';
    const rsensors = simInfo.bayRobot.sensors;
    for (ss = 0; ss < rsensors.length; ss++) {
      sensorString += '<br> id \'' + rsensors[ss].id + '\': ' +
        padnumber(rsensors[ss].value, 2);
    }
    if(averageHeapSize.length!=0) {
        document.getElementById('SensorLabel').innerHTML = sensorString
            + "<br>averageHeapSize = " + averageHeapSize.slice(-1).pop().toString()
            + "<br>percentageInAHeap = " + percentageInAHeap.slice(-1).pop().toString()
            + "<br>amountOfHeaps = " + amountOfHeaps.slice(-1).pop().toString()
            + "<br>groupDistribution = " + groupDistribution.slice(-1).pop().toString();
        //+ "<br>Amount of heaps: " + heaps.toString()
        //+ "<br>Amount of multiple-element heaps: " + (heaps - oneElementHeap).toString()
        //+ "<br>Average heap size: " + averSize.toString();
    }
  }
}

function setRobotNumber(newValue) {
  var n;
  while (robots.length > newValue) {
    n = robots.length - 1;
    Matter.World.remove(simInfo.world, robots[n].body);
    robots[n] = null;
    robots.length = n;
  }

  while (robots.length < newValue) {
    if (newValue > RobotInfo.length) {
      console.warn('You request '+newValue+' robots, but only ' + RobotInfo.length +
                   ' are defined in RobotInfo!');
      toggleSimulation();
      return;
    }
    n = robots.length;
    robots[n] = makeInteractiveElement(new InstantiateRobot(RobotInfo[n]),
                                       document.getElementById("arenaDidabot"));

    robots[n].onDrop = function(robot, event) {
      robot.isDragged = false;
    };

    robots[n].onDrag = function(robot, event) {
      	robot.isDragged = true;
        loadBay(robot);
        return true;
    };
  }
}


function padnumber(number, size) {
  if (number == Infinity) {
    return 'inf';
  }
  const s = "000000" + number;
  return s.substr(s.length - size);
}

function format(number) {
  // prevent HTML elements to jump around at sign flips etc
  return (number >= 0 ? '+' : '−') + Math.abs(number).toFixed(1);
}

function toggleSimulation() {
  simInfo.doContinue = !simInfo.doContinue;
  if (simInfo.doContinue) {
    Matter.Runner.start(simInfo.runner, simInfo.engine);
  }
  else {
    Matter.Runner.stop(simInfo.runner);
  }
}

function calculateGroups(positions, currentPos = null) {
  clusters = [];
  positionsLeft = Array.from(positions); //The array that contains the positions that haven't been assigned to a cluster yet

  while(positionsLeft.length != 0){
    thisCluster = [positionsLeft[0]];
    for (k=0;k<thisCluster.length;k++){
        closePositions = positionsLeft.filter(function(x){return (cartesianDis(x,thisCluster[k])< 40)});
        thisCluster = thisCluster.concat(closePositions);
        thisCluster = uniq(thisCluster);
    }
    clusters.push(thisCluster);
    positionsLeft = diff(positionsLeft,thisCluster);
  }
  return clusters;
}

/**
 * Removes all duplicate elements in an array
 */
function uniq(arr){
  return Array.from(new Set(arr));
}

/**
 * Computes the difference/complement between 2 arrays
 */
function diff(a1,a2){
    var result = [];
    for (var i = 0; i < a1.length; i++) {
        if (a2.indexOf(a1[i]) === -1) {
            result.push(a1[i]);
        }
    }
    for (i = 0; i < a2.length; i++) {
        if (a1.indexOf(a2[i]) === -1) {
            result.push(a2[i]);
        }
    }
    return Array.from(result);
}

function cartesianDis(posA,posB){
  return Math.sqrt( Math.pow(posA.x-posB.x,2) + Math.pow(posA.y-posB.y,2))
}

function sum(array){
  total = 0
  for (i in array){
    total += array[i]
  }
  return total
}

/**
 * Computes the average length of the nested arrays in an array
 */
function averLength(array){
    total = 0
    for (i in array){
        total += array[i].length
    }
    return total/array.length
}

function flatten(arr) {
    return arr.reduce(function (flat, toFlatten) {
        return flat.concat(Array.isArray(toFlatten) ? flatten(toFlatten) : toFlatten);
    }, []);
}
var saveAs=saveAs||function(e){"use strict";if(typeof e==="undefined"||typeof navigator!=="undefined"&&/MSIE [1-9]\./.test(navigator.userAgent)){return}var t=e.document,n=function(){return e.URL||e.webkitURL||e},r=t.createElementNS("http://www.w3.org/1999/xhtml","a"),o="download"in r,a=function(e){var t=new MouseEvent("click");e.dispatchEvent(t)},i=/constructor/i.test(e.HTMLElement)||e.safari,f=/CriOS\/[\d]+/.test(navigator.userAgent),u=function(t){(e.setImmediate||e.setTimeout)(function(){throw t},0)},s="application/octet-stream",d=1e3*40,c=function(e){var t=function(){if(typeof e==="string"){n().revokeObjectURL(e)}else{e.remove()}};setTimeout(t,d)},l=function(e,t,n){t=[].concat(t);var r=t.length;while(r--){var o=e["on"+t[r]];if(typeof o==="function"){try{o.call(e,n||e)}catch(a){u(a)}}}},p=function(e){if(/^\s*(?:text\/\S*|application\/xml|\S*\/\S*\+xml)\s*;.*charset\s*=\s*utf-8/i.test(e.type)){return new Blob([String.fromCharCode(65279),e],{type:e.type})}return e},v=function(t,u,d){if(!d){t=p(t)}var v=this,w=t.type,m=w===s,y,h=function(){l(v,"writestart progress write writeend".split(" "))},S=function(){if((f||m&&i)&&e.FileReader){var r=new FileReader;r.onloadend=function(){var t=f?r.result:r.result.replace(/^data:[^;]*;/,"data:attachment/file;");var n=e.open(t,"_blank");if(!n)e.location.href=t;t=undefined;v.readyState=v.DONE;h()};r.readAsDataURL(t);v.readyState=v.INIT;return}if(!y){y=n().createObjectURL(t)}if(m){e.location.href=y}else{var o=e.open(y,"_blank");if(!o){e.location.href=y}}v.readyState=v.DONE;h();c(y)};v.readyState=v.INIT;if(o){y=n().createObjectURL(t);setTimeout(function(){r.href=y;r.download=u;a(r);h();c(y);v.readyState=v.DONE});return}S()},w=v.prototype,m=function(e,t,n){return new v(e,t||e.name||"download",n)};if(typeof navigator!=="undefined"&&navigator.msSaveOrOpenBlob){return function(e,t,n){t=t||e.name||"download";if(!n){e=p(e)}return navigator.msSaveOrOpenBlob(e,t)}}w.abort=function(){};w.readyState=w.INIT=0;w.WRITING=1;w.DONE=2;w.error=w.onwritestart=w.onprogress=w.onwrite=w.onabort=w.onerror=w.onwriteend=null;return m}(typeof self!=="undefined"&&self||typeof window!=="undefined"&&window||this.content);if(typeof module!=="undefined"&&module.exports){module.exports.saveAs=saveAs}else if(typeof define!=="undefined"&&define!==null&&define.amd!==null){define("FileSaver.js",function(){return saveAs})}

function exportExcel()
{
    var data = [averageHeapSize,percentageInAHeap,amountOfHeaps];
    var keys = ['AverageHeapSize', 'percentageInAHeap', "amountOfHeaps"];       //Weights should still be added

    var convertToCSV = function(data, keys) {
        var orderedData = [];
        for (var i = 0, iLen = data.length; i < iLen; i++) {
            temp = data[i];
            for (var j = 0, jLen = temp.length; j < jLen; j++) {

                quotes = ['"'+temp[j]+'"'];
                if (!orderedData[j]) {
                    orderedData.push([quotes]);
                } else {
                    orderedData[j].push(quotes);
                }
            }
        }
        return keys.join(',') + '\r\n' + orderedData.join('\r\n');
    }


    var str = convertToCSV(data, keys);

    var blob = new Blob([str], {type: "text/plain;charset=utf-8"});
    d = new Date();
    var filename = "Hebbian Learning run at time " + d.toString();
    if(filename!=null && filename!="")
        saveAs(blob, [filename+'.csv']);
    else
        alert("please enter a filename!");

}
