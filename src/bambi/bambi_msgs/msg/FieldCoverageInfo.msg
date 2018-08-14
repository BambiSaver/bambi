float32 thermal_camera_ground_footprint_width
float32 thermal_camera_ground_footprint_height
# Scanning Altitude
float32 relative_altitude_scanning
# Altitude to use to get back home
float32 relative_altitude_returning
# both positions may be outside of the field borders
bambi_msgs/GeoPositionWithRelativeAltitude current_position
bambi_msgs/GeoPosition2D home_position
bambi_msgs/Field field