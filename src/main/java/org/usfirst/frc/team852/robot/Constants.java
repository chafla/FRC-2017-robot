package org.usfirst.frc.team852.robot;

public interface Constants {
    String CAMERA_GEAR_LOGGING_POSITION_TOPIC = "logging/camera/gear/alignment";
    String LIDAR_GEAR_LOGGING_POSITION_TOPIC = "logging/lidar/gear/distance";
    String LONG_LIDAR_LOGGING_POSITION_TOPIC = "logging/lidar/long/distance";
    String HEADING_LOGGING_POSITION_TOPIC = "logging/heading/degrees";
    String MQTT_TOPIC = "roborio/keyboard/command";
    String CAMERA_TOPIC = "camera/gear/x";
    String FRONT_LIDAR_TOPIC = "lidar/front/cm";
    String REAR_LIDAR_TOPIC = "lidar/rear/cm";
    String LEFT_LIDAR_TOPIC = "lidar/left/mm";
    String RIGHT_LIDAR_TOPIC = "lidar/right/mm";
    String MQTT_HOSTNAME = "mqtt-turtle.local"; /*"10.8.52.14";*/
    String HEADING_TOPIC = "heading/degrees";
    int MQTT_PORT = 1883;
}
