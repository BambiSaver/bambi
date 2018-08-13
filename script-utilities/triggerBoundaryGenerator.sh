#!/usr/bin/env bash


rostopic pub /bambi/mission_controller/trigger_boundary bambi_msgs/OrthoPhoto "filenameWithFullPath: '$BAMBI_OWNCLOUD_HOME/sample-border-1.kml'"

