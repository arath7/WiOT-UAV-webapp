<!DOCTYPE html>
<html lang="en">

<?php
  function runMyFunction() {
    echo 'I just ran a php function';
  }
  
  if ( isset($_GET['command']) ) {
    if($_GET['command']=='arm'){
      exec('python3 Scripts/BasicArdu/takeoffA.py');
    }
  }
?>

<head>
    <div class="header">
        <h1>Arducopter Drone Control</h1>
        <p> Institute for the Wireless Internet of Things </p>
        <link rel="stylesheet" href="site.css">


    </div>

  
  <link href="site.css" rel="stylesheet">
</head>

<body>

  <div id="main">
  <h1>Status</h1>

  <StatusButtons><buttonArm> <button class="button" onclick="location.href = 'indexFINAL.php?command=arm'">Arming</button></buttonArm>




  <alt href="movingA.py" class="button">Alt Button (currently displays link)</alt>

  <buttonDisarm><button class="button" onclick="alert('Disarming...')">Disarming</button></buttonDisarm></StatusButtons>





  <h2>Movement</h2>
  <button class="button" onclick="alert('Taking Off...')">Takeoff</button>
  <button class="button" onclick="alert('Landing...Stand Clear')">Landing</button> 
  
  <br>

  <button class="button" onclick="alert('Moving Up...')">Up</button>
  <button class="button" onclick="alert('Moving Down...')">Down</button>

  <br>

  <button class="button" onclick="alert('Moving Left...')">Left</button>
  <button class="button" onclick="alert('Moving Right...')">Right</button>

  <br> 
  
  <button class="button" onclick="alert('Moving in a Square...')">Square</button>
  <button class="button" onclick="alert('Moving Vertically...')">Vertical Movement</button>

  </div>

</body>




</html>