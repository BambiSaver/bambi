<?php

$dirPath = "img-data/";
$files = glob($dirPath . '*', GLOB_MARK);
foreach ($files as $file) {
    if (is_dir($file)) {
        self::deleteDir($file);
    } else {
        unlink($file);
    }
}

?>