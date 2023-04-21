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
    <div class="header">
        <h1>Arducopter Drone Control</h1>
        <p> Institute for the Wireless Internet of Things </p>
        <link rel="stylesheet" href="site.css">


    </div>

    <div class="navigation">
      <u1>
        <l1><a href="./index.php">Controls</a></li>
        <li><a href="./instructions.html">Instructions</a></li>
        <li><a href="./repo.html">Repository</a></li>
      </u1>
    </div>
    
  <link href="site.css" rel="stylesheet">
</head>



<body>

  <div id="main">
  <h1>Status</h1>

  <StatusButtons><buttonArm> <button class="button" onclick="location.href = 'indexFINAL.php?command=arm'">Arming</button></buttonArm>
  <buttonDisarm><button class="button" onclick="location.href = 'indexFINAL.php?command=disarm'">Disarming</button></buttonDisarm></StatusButtons>





  <h2>Movement</h2>
  <button class="button" onclick="location.href = 'indexFINAL.php?command=takeoff'">Takeoff</button>
  <button class="button" onclick="location.href = 'indexFINAL.php?command=landing'">Landing</button> 
  
  <br>

  <button class="button" onclick="location.href = 'indexFINAL.php?command=square'">Square Movement</button>
  <button class="button" onclick="location.href = 'indexFINAL.php?command=vertical'">Vertical Movement</button>

  <br>

  <button class="button" onclick="location.href = 'indexFINAL.php?command=left'">Left</button>
  <button class="button" onclick="location.href = 'indexFINAL.php?command=right'">Right</button>

  <br> 

  <button class="button" onclick="location.href = 'indexFINAL.php?command=front'">Front</button>
  <button class="button" onclick="location.href = 'indexFINAL.php?command=back'">Back</button>

  <br>

  <button class="button" onclick="location.href = 'indexFINAL.php?command=waypoint'">Waypoint</button>



  </div>

</body>

</html>