package org.usfirst.frc.team852.robot;

// Notice the 3 different ways of defining the MsgGenerator value
public enum SensorType {
    CAMERA_GEAR(Robot.CAMERA_GEAR_LOGGING_POSITION_TOPIC,
                (robot, desc) -> String.format("%d - %s",
                                               robot.getCurrentCameraGear().getX(),
                                               desc)),

    LIDAR_GEAR(Robot.LIDAR_GEAR_LOGGING_POSITION_TOPIC,
               (robot, logMsg) ->
                       String.format("Left: %dmm Right: %dmm - %s",
                                     robot.getCurrentLeftLidar().getMm(),
                                     robot.getCurrentRightLidar().getMm(),
                                     logMsg)),

    HEADING(Robot.HEADING_LOGGING_POSITION_TOPIC,
            (robot, logMsg) ->
                    String.format("Heading: %f Front: %dcm Rear: %dcm - %s",
                                  robot.getCurrentHeading().getDegree(),
                                  robot.getCurrentFrontLidar().getCm(),
                                  robot.getCurrentRearLidar().getCm(),
                                  logMsg)
    );


    final String topic;
    final MsgGenerator msgGenerator;

    SensorType(final String topic, final MsgGenerator msgGenerator) {
        this.topic = topic;
        this.msgGenerator = msgGenerator;
    }

    public String getTopic() {
        return this.topic;
    }

    public MsgGenerator getMsgGenerator() {
        return this.msgGenerator;
    }
}
