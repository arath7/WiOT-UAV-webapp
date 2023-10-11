<!DOCTYPE html>
<html lang="en">

<?php
  
  if ( isset($_GET['command']) ) {
    if($_GET['command']=='arm'){
      exec('python3 Scripts/BasicArdu/arming.py');
    }

    if($_GET['command']=='disarm'){
      exec('python3 Scripts/BasicArdu/disarming.py');
    }

    if($_GET['command']=='takeoff'){
      exec('python3 Scripts/BasicArdu/takeoff.py');
    }

    if($_GET['command']=='landing'){
      exec('python3 Scripts/BasicArdu/landing.py');
    }

    if($_GET['command']=='square'){
      exec('python3 Scripts/BasicArdu/squareMovement.py');
    }

    if($_GET['command']=='vertical'){
      exec('python3 Scripts/BasicArdu/vertMovement.py');
    }

    if($_GET['command']=='left'){
      exec('python3 Scripts/BasicArdu/moveLeft.py');
    }

    if($_GET['command']=='right'){
      exec('python3 Scripts/BasicArdu/moveRight.py');
    }

    if($_GET['command']=='front'){
      exec('python3 Scripts/BasicArdu/moveFront.py');
    }

    if($_GET['command']=='back'){
      exec('python3 Scripts/BasicArdu/moveBack.py');
    }

    if($_GET['command']=='waypoint'){
      exec('python3 Scripts/BasicArdu/waypointTesting.py');
    }
    
  }
?>

<head>
    <div class="mainheader">
        <h1>Arducopter Controls</h1>
        <h2> Institute for the Wireless Internet of Things </h2>
        <link rel="stylesheet" href="site.css">
    </div>

    <div class="navigationbar">
      <u1>
        <li><a href="./index.php">Controls</a></li>
        <li><a href="./instructions.html">Instructions</a></li>
        <li><a href="./repo.html">Repository</a></li>
      </u1>
    </div>
  <link href="site.css" rel="stylesheet">
</head>

<body>

  <div class="statusheading">
    <h1>Status</h1>
  </div>

  <div class="statusbuttons">
  <button class="button" onclick="location.href = 'index.php?command=arm'">Arming</button>
  <button class="button" onclick="location.href = 'index.php?command=disarm'">Disarming</button>
  </div>

  <br>

  <div class="movementheading">
    <h1>Movement</h1>
  </div>

  <div class="movementbuttons">
    <button class="button" onclick="location.href = 'index.php?command=takeoff'">Takeoff</button>
    <button class="button" onclick="location.href = 'index.php?command=landing'">Landing</button> 
    
    <br>
    <br>

    <button class="button" onclick="location.href = 'index.php?command=square'">Square Movement</button>
    <button class="button" onclick="location.href = 'index.php?command=vertical'">Vertical Movement</button>
    
    <br>
    <br>

    <button class="button" onclick="location.href = 'index.php?command=left'">Left</button>    
    <button class="button" onclick="location.href = 'index.php?command=right'">Right</button>
  
    <br> 
    <br>

    <button class="button" onclick="location.href = 'index.php?command=front'">Front</button>
    <button class="button" onclick="location.href = 'index.php?command=back'">Back</button>

    <br>
    <br>

    <button class="button" onclick="location.href = 'index.php?command=waypoint'">Waypoint</button>
  </div>





</body>
</html>
