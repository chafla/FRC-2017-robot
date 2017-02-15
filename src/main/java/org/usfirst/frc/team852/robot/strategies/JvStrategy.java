package org.usfirst.frc.team852.robot.strategies;

import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.data.CameraGearData;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;

import java.util.concurrent.atomic.AtomicReference;

import static org.usfirst.frc.team852.robot.SensorType.*;

public class JvStrategy implements Strategy {

    final AtomicReference<HeadingFeedback> headingFeedbackRef = new AtomicReference<>();

    @Override
    public void reset() {
        this.headingFeedbackRef.set(null);
    }

    public HeadingFeedback getHeadingFeedback() {
        return headingFeedbackRef.get();
    }

    @Override
    public void xboxAButtonPressed(final Robot robot) {
        final CameraGearData cameraGearData = robot.getCurrentCameraGear();

        if (cameraGearData == null) {
            System.out.println("Null CameraGearData");
            return;
        }
        if (cameraGearData.getTimestamp() <= robot.getCameraLastTime()) {
            System.out.println("Stale CameraGearData");
            return;
        } else {
            robot.updateCameraLastTime();
        }

        final int xVal = cameraGearData.getX();
        final int wVal = cameraGearData.getWidth();

        if (xVal == -1)
            System.out.println("No camera data");
        else if (xVal < (wVal / 2 - wVal * 0.1))
            robot.drive(.3, 0, 0, 0.1, CAMERA_GEAR, "turn left");
        else if (xVal > (wVal / 2 + wVal * 0.1))
            robot.drive(-0.3, 0, 0, 0.1, CAMERA_GEAR, "turn right");
        else
            robot.logMsg(CAMERA_GEAR, "centered");
    }

    @Override
    public void xboxBButtonPressed(final Robot robot) {
        // v1 of lidar driving
        final LidarData leftLidarData = robot.getCurrentLeftLidar();
        final LidarData rightLidarData = robot.getCurrentRightLidar();

        if (leftLidarData == null || rightLidarData == null) {
            System.out.println("Null LidarData");
            return;
        }

        if (leftLidarData.getTimestamp() <= robot.getLidarLeftLastTime()
                || rightLidarData.getTimestamp() <= robot.getLidarRightLastTime()) {
            System.out.println("Stale LidarData");
            return;
        } else {
            robot.updateLidarLeftLastTime();
            robot.updateLidarRightLastTime();
        }

        final int lVal = leftLidarData.getMm();
        final int rVal = rightLidarData.getMm();


        if (lVal == -1 || rVal == -1)
            System.out.println("Out of range");
        else if (lVal > 410 && rVal > 410)
            robot.drive(0, -0.3, 0, 0.1, LIDAR_GEAR, "forwards");
        else if (lVal < 390 && rVal < 390)
            robot.drive(0, 0.3, 0, 0.1, LIDAR_GEAR, "backwards");
        else if (lVal > rVal + 10)
            robot.drive(0, 0, -0.25, 0.1, LIDAR_GEAR, "rotate clockwise");
        else if (lVal < rVal - 10)
            robot.drive(0, 0, 0.25, 0.1, LIDAR_GEAR, "rotate counter-clockwise");
        else
            robot.logMsg(LIDAR_GEAR, "centered");
    }

    private static final double THRESHHOLD_DEGREES = 1.5;
    private static final double PID_CORRECTION = 0.1;

    @Override
    public void xboxXButtonPressed(final Robot robot) {
        final HeadingData headingData = robot.getCurrentHeading();

        if (headingData == null) {
            System.out.println("Null HeadingData");
            return;
        }

        if (headingData.getTimestamp() <= robot.getHeadingLastTime()) {
            System.out.println("Stale HeadingData");
            return;
        } else {
            robot.updateHeadingLastTime();
        }

        final double degrees = headingData.getDegree();

        // This will be set the first time through
        if (this.getHeadingFeedback() == null)
            this.headingFeedbackRef.set(new HeadingFeedback(degrees));

        final double errorDegrees = this.getHeadingFeedback().getError(degrees);

        final double turnSpeed;
        final String command;
        if (errorDegrees > THRESHHOLD_DEGREES) {
            // veered right, turn left
            turnSpeed = Math.max(errorDegrees * PID_CORRECTION, -1);
            command = "Forward and counter-clockwise";

        } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
            // veered left, turn right
            turnSpeed = Math.min(errorDegrees * PID_CORRECTION, 1);
            command = "Forward and clockwise";
        } else {
            // On course, drive straight.
            turnSpeed = 0;
            command = "Forward";
        }

        robot.drive(0, 0.3, turnSpeed, 0, HEADING, command);
    }

    @Override
    public void xboxYButtonPressed(final Robot robot) {
        // v2 of lidar driving
        final LidarData leftLidarData = robot.getCurrentLeftLidar();
        final LidarData rightLidarData = robot.getCurrentRightLidar();

        if (leftLidarData == null || rightLidarData == null) {
            System.out.println("Null LidarData");
            return;
        }
        if (leftLidarData.getTimestamp() <= robot.getLidarLeftLastTime()
                || rightLidarData.getTimestamp() <= robot.getLidarRightLastTime()) {
            System.out.println("Stale LidarData");
            return;
        } else {
            robot.updateLidarLeftLastTime();
            robot.updateLidarRightLastTime();
        }

        final int lVal = leftLidarData.getMm();
        final int rVal = rightLidarData.getMm();

        if (leftLidarData.getMm() == -1 || rightLidarData.getMm() == -1) {
            System.out.println("Out of range");
        } else if (lVal > rVal + 10) {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, -0.25, 0.1, LIDAR_GEAR, "clockwise and forward");
            else if (lVal < 280 && rVal < 280)
                robot.drive(0, 0.3, -0.25, 0.1, LIDAR_GEAR, "clockwise and backward");
            else
                robot.drive(0, 0, -0.25, 0.1, LIDAR_GEAR, "clockwise");
        } else if (lVal < rVal - 10) {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, 0.25, 0.1, LIDAR_GEAR, "counter-clockwise and forward");
            else if (lVal < 280 && rVal < 280)
                robot.drive(0, 0.3, 0.25, 0.1, LIDAR_GEAR, "counter-clockwise and backward");
            else
                robot.drive(0, 0, 0.25, 0.1, LIDAR_GEAR, "counter-clockwise");
        } else {
            if (lVal > 320 && rVal > 320)
                robot.drive(0, -0.3, 0, 0.1, LIDAR_GEAR, "forward");
            else if (lVal < 290 && rVal < 290)
                robot.drive(0, 0.3, 0, 0.1, LIDAR_GEAR, "backward");
        }
    }
}
