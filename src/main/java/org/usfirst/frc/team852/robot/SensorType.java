package org.usfirst.frc.team852.robot;

// Notice the 3 different ways of defining the MsgGenerator value
public enum SensorType {
    CAMERA_GEAR(Constants.CAMERA_GEAR_LOGGING_POSITION_TOPIC,
                (robot, desc) -> String.format("%d - %s",
                        robot.getStrategy().getCurrentCameraGear() != null ? robot.getStrategy().getCurrentCameraGear().getValOnce() : -1,
                                               desc)),

    LIDAR_GEAR(Constants.LIDAR_GEAR_LOGGING_POSITION_TOPIC,
               (robot, logMsg) ->
                       String.format("Left: %dmm Right: %dmm - %s",
                               robot.getStrategy().getCurrentLeftLidar() != null ? robot.getStrategy().getCurrentLeftLidar().getValOnce() : -1,
                               robot.getStrategy().getCurrentRightLidar() != null ? robot.getStrategy().getCurrentRightLidar().getValOnce() : -1,
                                     logMsg)),

    HEADING(Constants.HEADING_LOGGING_POSITION_TOPIC,
            (robot, logMsg) ->
                    String.format("Heading: %f Front: %dcm Rear: %dcm - %s",
                            robot.getStrategy().getCurrentHeading() != null ? robot.getStrategy().getCurrentHeading().getDegreesOnce() : -1,
                            robot.getStrategy().getCurrentFrontLidar() != null ? robot.getStrategy().getCurrentFrontLidar().getValOnce() : -1,
                            robot.getStrategy().getCurrentRearLidar() != null ? robot.getStrategy().getCurrentRearLidar().getValOnce() : -1,
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
