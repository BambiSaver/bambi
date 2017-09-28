
<nav id="mainMenu" class="navbar navbar-expand-lg navbar-light bg-light">
  <a class="navbar-brand" href="#">Masking Webapp</a>
  <button class="navbar-toggler" type="button" data-toggle="collapse" data-target="#navbarSupportedContent" aria-controls="navbarSupportedContent" aria-expanded="false" aria-label="Toggle navigation">
    <span class="navbar-toggler-icon"></span>
  </button>

  <div class="collapse navbar-collapse" id="navbarSupportedContent">
    <ul class="navbar-nav mr-auto">
      <li class="nav-item <?php if (basename($_SERVER["SCRIPT_FILENAME"], '.php') == "index") echo "active"; ?>">
        <a class="nav-link" href="index.php">Masking</a>
      </li>
      <li class="nav-item <?php if (basename($_SERVER["SCRIPT_FILENAME"], '.php') == "upload") echo "active"; ?>">
        <a class="nav-link" href="upload.php">Upload</a>
      </li>
      <li class="nav-item <?php if (basename($_SERVER["SCRIPT_FILENAME"], '.php') == "view") echo "active"; ?>">
        <a class="nav-link" href="view.php">View</a>
      </li>
    </ul>
  </div>
</nav>

