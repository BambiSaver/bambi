<!doctype html>

<html lang="en">
<head>
  <meta charset="utf-8">

  <title>Masking Webapp</title>
  <meta name="description" content="Masking the orthophotos for Neural Learning Algorithm">
  <meta name="author" content="Florian Mahlknecht">

  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
  <link rel="stylesheet" href="css/style.css">

  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0-beta/css/bootstrap.min.css" integrity="sha384-/Y6pD6FV/Vv2HJnA6t+vslU6fwYXjCFtcEpHbNJ0lyAFsXTsjBbfaDjzALeQsN6M" crossorigin="anonymous">
</head>

<body>

  <?php include("menu.php"); ?>

  <form id="maskSubmitForm">
    <input class="btn btn-primary" type="submit" value="Submit">
  </form>

  <div id="canvas-wrapper" class="wrapper">
  </div>


  <script src="https://code.jquery.com/jquery-3.2.1.min.js" crossorigin="anonymous"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.11.0/umd/popper.min.js" crossorigin="anonymous"></script>
  <script src="https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0-beta/js/bootstrap.min.js" crossorigin="anonymous"></script>
  <script src="js/script.js"></script>
</body>
</html>