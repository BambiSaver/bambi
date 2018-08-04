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

  <!-- Latest compiled and minified CSS -->
  <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css">

</head>

<body>

  <?php include("menu.php"); ?>


  <ul class="pagination">
  <?php
    //.*\.(png|jpg)

    $id = intval($_GET["id"] ?? 1);

    $fileList = glob("img-data/*");

    //echo '<img src="' . $fileList[$id - 1] . '">';

    /*foreach ($fileList as $filename) {
        echo "<p>$filename size " . filesize($filename) . "</p>\n";
    }*/

    for ($i=1; $i <= count($fileList); ++$i) {
      if ($i == $id) {
        echo '<li><a class="active" href="index.php?id=' . $i . '">' . $i . '</a></li>';
      } else {
        echo '<li><a href="index.php?id=' . $i . '">' . $i . '</a></li>';
      }
    }
  ?>
  </ul>

  <form id="maskSubmitForm">
    <input class="btn btn-primary" type="submit" value="Submit">
  </form>

  <div id="canvas-wrapper" class="wrapper">
  </div>


  <!-- jQuery library -->
  <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js"></script>
<!-- Latest compiled JavaScript -->
  <script src="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.11.0/umd/popper.min.js" crossorigin="anonymous"></script>
  <script src="js/script.js"></script>
</body>
</html>