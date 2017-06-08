package org.usfirst.frc.team852.robot.strategy;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.data.CameraData;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;
import org.usfirst.frc.team852.robot.navigation.HeadingError;
import org.usfirst.frc.team852.robot.navigation.PIDControl;

import static org.usfirst.frc.team852.robot.SensorType.*;

public class JvStrategyPID extends Strategy {

    private static final double THRESHHOLD_DEGREES = 1.5;
    private static final double PID_CORRECTION_STRAIGHT = 0.01;
    private static final double PID_CORRECTION_TURN = 0.1;

    private double x = 0;
    private double y = 0;
    private double rot = 0;
    private static final int upperLidarThreshold = 570; // old 640
    private static final int lowerLidarThreshold = 520; // old 600
    private static double xSpeed = 0.075;
    private static double ySpeed = 0.1;
    private static double rotSpeed = 0.075;
    private static double turningSpeed = 0.175;

    private double timeSinceLastMovementXboxY = 0.0;

    private PIDControl lidarGearAlignPID;
    private PIDControl lidarGearDistPID;
    private PIDControl cameraPID;
    private PIDControl headingPID;
    private PIDControl rearLidarPID;


    public JvStrategyPID(final Robot robot) {
        super(robot);
        this.reset();

        // Because we're dealing with very low margins of adjustment ([-1.0, 1.0] when compared with the errors (might be in the thousands)
        // the PID values are super low

        this.lidarGearAlignPID = new PIDControl(0.00075,0.00001,0.00000);
        //this.lidarGearAlignPID = new PIDControl(0,0,0);
        this.lidarGearAlignPID.setOutputConstraints(-1, 1);
        this.lidarGearAlignPID.setReversed();

//        this.lidarGearDistPID = new PIDControl(0.00125,0,0);  // too fast
        this.lidarGearDistPID = new PIDControl(0.0009,0.000006,0.00000);
        this.lidarGearDistPID.setReversed();  // Needs to be reversed due to the way that the wheels and controllers are set up
        this.lidarGearDistPID.setOutputConstraints(-1, 1);

        this.cameraPID = new PIDControl(0,0,0);
        this.headingPID = new PIDControl(0,0,0);
        this.rearLidarPID = new PIDControl(0,0,0);


    }

    @Override
    public void reset() {
        this.resetHeadingError();
        //this.lidarGearAlignPID.reset();
        //this.lidarGearDistPID.reset();
    }



    // align robot with peg
    @Override
    public void onXboxA() {
        final Robot robot = this.getRobot();
        final CameraData cameraData = this.getCurrentCameraGear();
        final LidarData leftLidarData = this.getCurrentLeftLidar();
        final LidarData rightLidarData = this.getCurrentRightLidar();

        if (cameraData == null) {
            robot.rumble(1);
            robot.logMsg(CAMERA_GEAR, "Null camera data");
            robot.stopRandP();
            edu.wpi.first.wpilibj.Timer.delay(0.1);
            return;
        }

        if (leftLidarData == null) {
            robot.rumble(1);
            robot.logMsg(LIDAR_GEAR, "Null left lidar data");
            robot.stopRandP();
            edu.wpi.first.wpilibj.Timer.delay(0.1);
            return;
        }

        if (rightLidarData == null) {
            robot.rumble(1);
            robot.logMsg(LIDAR_GEAR, "Null right lidar data");
            robot.stopRandP();
            edu.wpi.first.wpilibj.Timer.delay(0.1);
            return;
        }

        final int lVal = leftLidarData.getValOnce();
        final int rVal = rightLidarData.getValOnce();
        final int xVal = cameraData.getValOnce();
        final int wVal = cameraData.getWidth();
        System.out.println("right lidar: " + rVal + ", left lidar: " + lVal + ", x: " + xVal);

        if (xVal == -1) {
            robot.logMsg(CAMERA_GEAR, "No camera data");
            robot.rumble(1);
        } else if (xVal < ((wVal / 2 - 25) - wVal * 0.075)) {
            x = xSpeed;
            robot.stopRandP();
        } else if (xVal > ((wVal / 2 - 25) + wVal * 0.075)) {
            x = -xSpeed;
            robot.stopRandP();
        } else {
            if (x != 0)
                x = 0;
        }

        if (leftLidarData.getValOnce() == -1 || rightLidarData.getValOnce() == -1) {
            robot.logMsg(LIDAR_GEAR, "Out of range");
            robot.rumble(1);
        } else if (lVal > rVal + 10) {
            if (rot != rotSpeed)
                rot = rotSpeed;
            if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -ySpeed)
                y = -ySpeed;
            else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != ySpeed)
                y = ySpeed;
            else {
                if (y != 0)
                    y = 0;
            }
        } else if (lVal < rVal - 10) {
            if (rot != -rotSpeed)
                rot = -rotSpeed;
            if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -ySpeed)
                y = -ySpeed;
            else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != ySpeed)
                y = ySpeed;
            else {
                if (y != 0)
                    y = 0;
            }
        } else {
            if (rot != 0)
                rot = 0;
            if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -ySpeed)
                y = -ySpeed;
            else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != ySpeed)
                y = ySpeed;
            else {
                if (y != 0)
                    y = 0;
            }
        }

        robot.drive(x, y, rot, CAMERA_GEAR, x + " " + y + " " + rot);
        robot.rumble(0);

        if (x == 0 && y == 0 && rot == 0) {
            if (xVal < ((wVal / 2 - 25) - 1))
                robot.moveRandPRight();
            else if (xVal > ((wVal / 2 - 25) + 1))
                robot.moveRandPLeft();
            else
                robot.stopRandP();
        }
    }

    // camera alignment
    @Override
    public void onXboxB() {
        final Robot robot = this.getRobot();
        final CameraData cameraData = this.getCurrentCameraGear();

        if (cameraData == null) {
            robot.logMsg(CAMERA_GEAR, "Null camera data");
            robot.stopRandP();
            return;
        }

        final int xVal = cameraData.getValOnce();
        final int wVal = cameraData.getWidth();

        if (xVal == -1) {
            robot.logMsg(CAMERA_GEAR, "No camera data");
            robot.stopRandP();
        } else if (xVal < (wVal / 2 - wVal * 0.1)) {
            robot.drive(x, 0, 0, CAMERA_GEAR, "move left");
            robot.stopRandP();
        } else if (xVal > (wVal / 2 + wVal * 0.1)) {
            robot.drive(-x, 0, 0, CAMERA_GEAR, "move right");
            robot.stopRandP();
        } else {
            if (xVal < (wVal / 2 - 1))
                robot.moveRandPRight();
            else if (xVal > (wVal / 2 + 1))
                robot.moveRandPLeft();
            else
                robot.stopRandP();
        }
    }

    // goByRear forward with corrections from heading
    @Override
    public void onXboxX() {
        this.lidarGearAlignPID.reset();
        this.lidarGearDistPID.reset();
        /*
        final Robot robot = this.getRobot();
        final HeadingData headingData = this.getCurrentHeading();

        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }


        if (headingData.isInvalid()) {
            //robot.logMsg(HEADING, headingData.getAlreadyReadMsg());
            //robot.drive(0, -0.2, 0, HEADING, "old data -> goByRear straight");
            //return;
        }

        final double degrees = headingData.getDegreesOnce();

        // This will be set the first time through
        if (this.getHeadingError() == null)
            this.setHeadingError(new HeadingError(degrees));

        final double errorDegrees = this.getHeadingError().getError(degrees);

        final double turnSpeed;
        final String command;
        if (errorDegrees > THRESHHOLD_DEGREES) {
            // veered right, turn left, turnSpeed will be no less than -1
            turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
            command = "Forward and counter-clockwise";

        } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
            // veered left, turn right, turnSpeed will be no more 1
            turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
            command = "Forward and clockwise";
        } else {
            // On course, drive straight.
            turnSpeed = 0;
            command = "Forward";
        }
        robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
        robot.drive(0, -0.3, turnSpeed, HEADING, command);
*/
//        final Robot robot = this.getRobot();
//        final CameraData cameraData = this.getCurrentCameraGear();
//        final HeadingData headingData = this.getCurrentHeading();
//        final LidarData frontLidarData = this.getCurrentFrontLidar();
//        final LidarData rearLidarData = this.getCurrentRearLidar();
//        final LidarData leftLidarData = this.getCurrentLeftLidar();
//        final LidarData rightLidarData = this.getCurrentRightLidar();
//
//        if (cameraData == null) {
//            robot.logMsg(CAMERA_GEAR, "Null camera data");
//            robot.stopRandP();
//            return;
//        }
//        if (leftLidarData == null) {
//            robot.logMsg(LIDAR_GEAR, "Null left lidar data");
//            robot.stopRandP();
//            return;
//        }
//        if (rightLidarData == null) {
//            robot.logMsg(LIDAR_GEAR, "Null right lidar data");
//            robot.stopRandP();
//            return;
//        }
//        if (headingData == null) {
//            robot.logMsg(HEADING, "Null heading data");
//            robot.stopRandP();
//            return;
//        }
//        if (frontLidarData == null) {
//            robot.logMsg(HEADING, "Null heading data");
//            robot.stopRandP();
//            return;
//        }
//        if (rearLidarData == null) {
//            robot.logMsg(HEADING, "Null heading data");
//            robot.stopRandP();
//            return;
//        }
//
//        final double fVal = frontLidarData.getValOnce();
//        final double bVal = rearLidarData.getValOnce();
//        final int lVal = leftLidarData.getValOnce();
//        final int rVal = rightLidarData.getValOnce();
//        final int xVal = cameraData.getValOnce();
//        final int wVal = cameraData.getWidth();
//
//        final double degrees = headingData.getDegreesOnce();
//        // This will be set the first time through
//        if (this.getHeadingError() == null)
//            this.setHeadingError(new HeadingError(degrees));
//
//        final double errorDegrees = this.getHeadingError().getError(degrees);
//
//        final double turnSpeed;
//        final String command;
//        if (fVal > frontTarget && bVal < rearTarget) {
//            if (errorDegrees > THRESHHOLD_DEGREES) {
//                // veered right, turn left, turnSpeed will be no less than -1
//                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
//                command = "Forward and counter-clockwise";
//
//            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
//                // veered left, turn right, turnSpeed will be no more 1
//                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
//                command = "Forward and clockwise";
//            } else {
//                // On course, drive straight.
//                turnSpeed = 0;
//                command = "Forward";
//            }
//            robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
//            robot.drive(0, -0.3, turnSpeed, HEADING, command);
//            return;
//        }
//
//        this.setHeadingError(new HeadingError(degrees + turn));
//
//        if (errorDegrees > THRESHHOLD_DEGREES || errorDegrees < -THRESHHOLD_DEGREES) {
//            if (errorDegrees > THRESHHOLD_DEGREES) {
//                // veered right, turn left, turnSpeed will be no less than -1
//                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
//                command = "Counter-clockwise";
//
//            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
//                // veered left, turn right, turnSpeed will be no more 1
//                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
//                command = "Clockwise";
//            } else {
//                // On course, drive straight.
//                turnSpeed = 0;
//                command = "Aligned";
//            }
//
//            robot.drive(0, 0, turnSpeed, HEADING, command);
//            return;
//        }
//
//        if ((rVal > 1200 || rVal == -1) || (lVal > 1200 || lVal == -1)) {
//            if (errorDegrees > THRESHHOLD_DEGREES) {
//                // veered right, turn left, turnSpeed will be no less than -1
//                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
//                command = "Forward and counter-clockwise";
//
//            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
//                // veered left, turn right, turnSpeed will be no more 1
//                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
//                command = "Forward and clockwise";
//            } else {
//                // On course, drive straight.
//                turnSpeed = 0;
//                command = "Forward";
//            }
//            robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
//            robot.drive(0, -0.5, turnSpeed, HEADING, command);
//            return;
//        }
//        // change to drive using heading not lidar
//        if (lVal > 510 && rVal > 510) {
//
//            if (errorDegrees > THRESHHOLD_DEGREES) {
//                // veered right, turn left, turnSpeed will be no less than -1
//                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
//                command = "Forward and counter-clockwise";
//
//            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
//                // veered left, turn right, turnSpeed will be no more 1
//                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
//                command = "Forward and clockwise";
//            } else {
//                // On course, drive straight.
//                turnSpeed = 0;
//                command = "Forward";
//            }
//            robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
//            robot.drive(0, -0.5 * (lVal + rVal) / 2000, turnSpeed, HEADING, command);
//            return;
//
//        }
//
//        if (xVal < (wVal / 2 - wVal * 0.075) || xVal > (wVal / 2 + wVal * 0.075)) {
//            if (xVal == -1) {
//                robot.logMsg(CAMERA_GEAR, "No camera data");
//                robot.rumble(1);
//            } else if (xVal < (wVal / 2 - wVal * 0.075)) {
//                x = 0.15;
//                robot.stopRandP();
//            } else if (xVal > (wVal / 2 + wVal * 0.075)) {
//                x = -0.15;
//                robot.stopRandP();
//            } else {
//                if (x != 0)
//                    x = 0;
//            }
//
//            robot.drive(x, 0, 0, CAMERA_GEAR, x + " " + y + " " + rot);
//
//            if (x == 0 && y == 0 && rot == 0) {
//                if (xVal < (wVal / 2 - 1))
//                    robot.moveRandPRight();
//                else if (xVal > (wVal / 2 + 1))
//                    robot.moveRandPLeft();
//                else
//                    robot.stopRandP();
//            }
//            return;
//        }
//        robot.pushGear();
//        edu.wpi.first.wpilibj.Timer.delay(0.5);
//        robot.retractPiston();
//        final Robot robot = this.getRobot();
//        final HeadingData headingData = this.getCurrentHeading();
//
//        if (headingData == null) {
//            robot.logMsg(HEADING, "Null heading data");
//            return;
//        }
//
//        if (headingData.isInvalid()) {
//            //System.out.println(headingData.getAlreadyReadMsg());
//            //this.waitOnHeading(0);
//            //return;
//        }
//
//        final double degrees = headingData.getDegreesOnce();
//
//        // This will be set the first time through
//        if (this.getHeadingError() == null)
//            this.setHeadingError(new HeadingError(degrees + turn));
//
//        final double errorDegrees = this.getHeadingError().getError(degrees);
//
//        final double turnSpeed;
//        final String command;
//        if (errorDegrees > THRESHHOLD_DEGREES) {
//            // veered right, turn left, turnSpeed will be no less than -1
//            turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_TURN, -0.2);
//            command = "Counter-clockwise";
//
//        } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
//            // veered left, turn right, turnSpeed will be no more 1
//            turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_TURN, 0.2);
//            command = "Clockwise";
//        } else {
//            // On course, drive straight.
//            turnSpeed = 0;
//            command = "Aligned";
//        }
//
//        robot.drive(0, 0, turnSpeed, HEADING, command);
    }

    // lidar alignment
    @Override
    public void onXboxY() {
        // TODO Add ability to tuning class to change PID values on the fly w/ smartdashboard
        final Robot robot = this.getRobot();
        final LidarData leftLidarData = this.getCurrentLeftLidar();
        final LidarData rightLidarData = this.getCurrentRightLidar();

        if (leftLidarData == null) {
            robot.logMsg(LIDAR_GEAR, "Null left lidar data");
            return;
        }

        if (rightLidarData == null) {
            robot.logMsg(LIDAR_GEAR, "Null right lidar data");
            return;
        }

        final int lVal = leftLidarData.getValOnce();
        final int rVal = rightLidarData.getValOnce();

        // lidar driving, turn while moving forwards

        if (lVal == -1 || rVal == -1) {
            robot.logMsg(LIDAR_GEAR, "Out of range");
            return;
        }


        // Get the rotation and distance to travel based on the error.
        double lidarAdjustmentRot = lidarGearAlignPID.getPID(rVal - lVal);
        double lidarAdjustmentDist = lidarGearDistPID.getPID(600, (lVal + rVal) / 2);

        String distMsg;
        String turnMsg;

        if (lidarAdjustmentRot > 0)
            turnMsg = "Clockwise";
        else if (lidarAdjustmentRot < 0)
            turnMsg = "Counterclockwise";
        else
            turnMsg = "Centered";

        if (lidarAdjustmentDist > 0)
            distMsg = "Forwards";
        else if (lidarAdjustmentDist < 0)
            distMsg = "Reverse";
        else
            distMsg = "Centered";

        System.out.println(String.format("%s and %s, dist error: %s, rot error: %s, cur dist: %s", turnMsg, distMsg, lidarAdjustmentDist, lidarAdjustmentRot, (lVal + rVal) / 2));

        robot.drive(0, -lidarAdjustmentDist, -lidarAdjustmentRot, LIDAR_GEAR, String.format("%s and %s", turnMsg, distMsg));

    }

    // move rack and pinion left
    @Override
    public void onXboxLB() {
        this.getRobot().moveRandPLeft();
    }

    // move rack and pinion right
    @Override
    public void onXboxRB() {
        this.getRobot().moveRandPRight();
    }

    // turn using heading
    @Override
    public void onXboxBack() {
        this.getRobot().centerRandP();
    }

    // move piston forward while holding, on release retract
    @Override
    public void onXboxStart() {
        this.lidarGearDistPID.reset();
        this.lidarGearAlignPID.reset();
    }

    // retract piston
    @Override
    public void onXboxLS() {
        this.getRobot().retractPiston();
    }

    // push gear
    @Override
    public void onXboxRS() {
        this.getRobot().pushGear();
    }

    // autonomous methods
    @Override
    public void goByRear(int dist) {
        iterationInit();
        final Robot robot = this.getRobot();

        double forwardSpeed;
        double turnSpeed;

        this.rearLidarPID.reset();  // Needed to be called
        this.headingPID.reset();

        LidarData rearLidarData = this.getCurrentRearLidar();

        if (rearLidarData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }

        final double startVal = rearLidarData.getValOnce();

        while (robot.isEnabled() && robot.isAutonomous()) {
            iterationInit();

            final HeadingData headingData = this.getCurrentHeading();
            rearLidarData = this.getCurrentRearLidar();
            if (headingData == null) {
                robot.logMsg(HEADING, "Null heading data");
                return;
            }

            final double rearLidarDist = rearLidarData.getValOnce();
            final double degrees = headingData.getDegreesOnce();
            // This will be set the first time through
            if (this.getHeadingError() == null)
                this.setHeadingError(new HeadingError(degrees));

            final double errorDegrees = this.getHeadingError().getError(degrees);

            forwardSpeed = this.rearLidarPID.getPID(errorDegrees);
            turnSpeed = this.headingPID.getPID(dist, rearLidarDist);



            //final String command;


            robot.drive(0, forwardSpeed, turnSpeed, HEADING, "Following bearing.");


            /*
            if (rearLidarDist - startVal < dist) {
                if (errorDegrees > THRESHHOLD_DEGREES) {
                    // veered right, turn left, turnSpeed will be no less than -1
                    turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.1);
                    command = "Forward and counter-clockwise";

                } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                    // veered left, turn right, turnSpeed will be no more 1
                    turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.1);
                    command = "Forward and clockwise";
                } else {
                    // On course, drive straight.
                    turnSpeed = 0;
                    command = "Forward";
                }
                robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
                System.out.println(String.format("rear lidar: %f %f %f %f %f %s", (rearLidarDist - startVal), rearLidarDist, startVal, errorDegrees, turnSpeed, command));
                robot.drive(0, -0.4, 0, HEADING, command);
            } else
                return;
                */
            edu.wpi.first.wpilibj.Timer.delay(0.005);
        }
    }

    // don't use this
    @Override
    public void goByFront(int dist) {
        iterationInit();
        final Robot robot = this.getRobot();

        LidarData frontLidarData = this.getCurrentFrontLidar();

        if (frontLidarData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }

        final double startVal = frontLidarData.getValOnce();

        while (robot.isEnabled() && robot.isAutonomous()) {
            iterationInit();

            final HeadingData headingData = this.getCurrentHeading();
            frontLidarData = this.getCurrentFrontLidar();
            if (headingData == null) {
                robot.logMsg(HEADING, "Null heading data");
                return;
            }

            final double fVal = frontLidarData.getValOnce();
            final double degrees = headingData.getDegreesOnce();
            // This will be set the first time through
            if (this.getHeadingError() == null)
                this.setHeadingError(new HeadingError(degrees));

            final double errorDegrees = this.getHeadingError().getError(degrees);

            final double turnSpeed;
            final String command;
            if (startVal - fVal < dist) {
                if (errorDegrees > THRESHHOLD_DEGREES) {
                    // veered right, turn left, turnSpeed will be no less than -1
                    turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                    command = "Forward and counter-clockwise";

                } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                    // veered left, turn right, turnSpeed will be no more 1
                    turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                    command = "Forward and clockwise";
                } else {
                    // On course, drive straight.
                    turnSpeed = 0;
                    command = "Forward";
                }
                robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
                robot.drive(0, -ySpeed, turnSpeed, HEADING, command);
            } else
                return;
            edu.wpi.first.wpilibj.Timer.delay(0.005);
        }
    }

    @Override
    public void turn(double degreeTurn) {
        iterationInit();
        final Robot robot = this.getRobot();

        this.setHeadingError(null);
        while (robot.isEnabled() && robot.isAutonomous()) {
            iterationInit();
            final HeadingData headingData = this.getCurrentHeading();
            if (headingData == null) {
                robot.logMsg(HEADING, "Null heading data");
                System.out.println("null");
                return;
            }
            final double degrees = headingData.getDegreesOnce();

            // This will be set the first time through
            if (this.getHeadingError() == null) {
                this.setHeadingError(new HeadingError(degrees));
            }


            final double errorDegrees = this.getHeadingError().getError(degrees);

            System.out.println(String.format("current error: %f %f %f %f",
                    errorDegrees, degreeTurn, this.getHeadingError().getInitialHeading(), degrees));
            final double turnSpeed;
            final String command;

            if (degreeTurn > 0) {
                if (errorDegrees < degreeTurn) {
                    turnSpeed = turningSpeed;
                    command = "Clockwise " + errorDegrees;
                    robot.drive(0, 0, turnSpeed, HEADING, command);
                } else {
                    turnSpeed = 0;
                    command = "Aligned " + errorDegrees;
                    robot.drive(0, 0, turnSpeed, HEADING, command);
                    return;
                }
            } else {
                if (Math.abs(errorDegrees) < Math.abs(degreeTurn)) {
                    turnSpeed = -turningSpeed;
                    command = "Counter-clockwise " + errorDegrees;
                    robot.drive(0, 0, turnSpeed, HEADING, command);
                } else {
                    turnSpeed = 0;
                    command = "Aligned " + errorDegrees;
                    robot.drive(0, 0, turnSpeed, HEADING, command);
                    return;
                }
            }
            edu.wpi.first.wpilibj.Timer.delay(0.005);
        }
    }

    @Override
    public void goUntilLocatedWall() {
        iterationInit();
        final Robot robot = this.getRobot();
        this.setHeadingError(null);
        while (robot.isEnabled() && robot.isAutonomous()) {
            iterationInit();
            final HeadingData headingData = this.getCurrentHeading();
            final LidarData leftLidarData = this.getCurrentLeftLidar();
            final LidarData rightLidarData = this.getCurrentRightLidar();

            if (headingData == null) {
                robot.logMsg(HEADING, "Null heading data");
                return;
            }
            if (leftLidarData == null) {
                robot.logMsg(LIDAR_GEAR, "Null left lidar data");
                robot.stopRandP();
                return;
            }
            if (rightLidarData == null) {
                robot.logMsg(LIDAR_GEAR, "Null right lidar data");
                robot.stopRandP();
                return;
            }
            final int lVal = leftLidarData.getValOnce();
            final int rVal = rightLidarData.getValOnce();
            final double degrees = headingData.getDegreesOnce();
            // This will be set the first time through
            if (this.getHeadingError() == null)
                this.setHeadingError(new HeadingError(degrees));

            final double errorDegrees = this.getHeadingError().getError(degrees);

            System.out.println("left lidar: " + lVal + ", right lidar: " + rVal);

            final double turnSpeed;
            final String command;
            if ((rVal > 1200 || rVal == -1) || (lVal > 1200 || lVal == -1)) {
                if (errorDegrees > THRESHHOLD_DEGREES) {
                    // veered right, turn left, turnSpeed will be no less than -1
                    turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                    command = "Forward and counter-clockwise";

                } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                    // veered left, turn right, turnSpeed will be no more 1
                    turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                    command = "Forward and clockwise";
                } else {
                    // On course, drive straight.
                    turnSpeed = 0;
                    command = "Forward";
                }
                robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
                robot.drive(0, -ySpeed * 3, turnSpeed, HEADING, String.format("%s,%d,%d", command, lVal, rVal));
            } else {
                command = "stop";
                robot.drive(0, 0, 0, HEADING, String.format("%s,%d,%d", command, lVal, rVal));
                return;
            }
        }
    }

    @Override
    public void goUntilTargetDistance() {
        iterationInit();
        final Robot robot = this.getRobot();
        this.setHeadingError(null);
        while (robot.isEnabled() && robot.isAutonomous()) {
            iterationInit();
            final HeadingData headingData = this.getCurrentHeading();
            final LidarData leftLidarData = this.getCurrentLeftLidar();
            final LidarData rightLidarData = this.getCurrentRightLidar();

            if (headingData == null) {
                robot.logMsg(HEADING, "Null heading data");
                return;
            }
            if (leftLidarData == null) {
                robot.logMsg(LIDAR_GEAR, "Null left lidar data");
                robot.stopRandP();
                return;
            }
            if (rightLidarData == null) {
                robot.logMsg(LIDAR_GEAR, "Null right lidar data");
                robot.stopRandP();
                return;
            }
            final int lVal = leftLidarData.getValOnce();
            final int rVal = rightLidarData.getValOnce();
            final double degrees = headingData.getDegreesOnce();
            // This will be set the first time through
            if (this.getHeadingError() == null)
                this.setHeadingError(new HeadingError(degrees));

            System.out.println("left lidar: " + lVal + ", right lidar: " + rVal);

            final double errorDegrees = this.getHeadingError().getError(degrees);

            final double turnSpeed;
            final String command;
            if (lVal > upperLidarThreshold - 20 && rVal > upperLidarThreshold - 20) {

                if (errorDegrees > THRESHHOLD_DEGREES) {
                    // veered right, turn left, turnSpeed will be no less than -1
                    turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                    command = "Forward and counter-clockwise";

                } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                    // veered left, turn right, turnSpeed will be no more 1
                    turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                    command = "Forward and clockwise";
                } else {
                    // On course, drive straight.
                    turnSpeed = 0;
                    command = "Forward";
                }
                robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
                robot.drive(0, -ySpeed * 1.5, turnSpeed, HEADING, String.format("%s, %d, %d", command, lVal, rVal));
            } else {
                command = "stop";
                robot.drive(0, 0, 0, HEADING, String.format("%s,%d,%d", command, lVal, rVal));
                return;
            }
        }
    }

    @Override
    public void center() {
        final Robot robot = this.getRobot();

        while (robot.isEnabled() && robot.isAutonomous()) {
            iterationInit();
            final CameraData cameraData = this.getCurrentCameraGear();

            if (cameraData == null) {
                robot.logMsg(HEADING, "Null camera data");
                System.out.println("Null data");
                continue;
            }
            final int xVal = cameraData.getValOnce();
            final int wVal = cameraData.getWidth();

            System.out.println("x: " + xVal + ", width: " + wVal);

            if (xVal < ((wVal / 2 - wVal - 25) * 0.075) || xVal > ((wVal / 2 - 25) + wVal * 0.075)) {
                if (xVal == -1) {
                    System.out.println("Camera data is -1");
                    robot.logMsg(CAMERA_GEAR, "No camera data");
                    robot.rumble(1);
                    x = 0;
                } else if (xVal < ((wVal / 2 - 25) - wVal * 0.075)) {
                    x = xSpeed;
                    System.out.println("right " + x);
                    robot.stopRandP();
                } else if (xVal > ((wVal / 2 - 25) + wVal * 0.075)) {
                    x = -xSpeed;
                    System.out.println("left " + x);
                    robot.stopRandP();
                } else {
                    if (x != 0)
                        x = 0;
                    System.out.println("stop");
                }

                robot.drive(x * 1.5, 0, 0, CAMERA_GEAR, x + " " + y + " " + rot);

                if (x == 0 && y == 0 && rot == 0 && xVal != -1) {
                    if (xVal < ((wVal / 2 - 25) - 1))
                        robot.moveRandPRight();
                    else if (xVal > ((wVal / 2 - 25) + 1))
                        robot.moveRandPLeft();
                    else {
                        robot.stopRandP();
                        robot.pushGear();
                        edu.wpi.first.wpilibj.Timer.delay(0.5);
                        robot.retractPiston();
                        return;
                    }
                }
            } else if (x == 0 && (xVal < ((wVal / 2 - 25) - 1) || xVal > ((wVal / 2 - 25) + 1)) && xVal != -1) {
                if (xVal < ((wVal / 2 - 25) - 1))
                    robot.moveRandPRight();
                else if (xVal > ((wVal / 2 - 25) + 1))
                    robot.moveRandPLeft();
                else
                    robot.stopRandP();
            } else if (xVal == -1) {
                x = -xSpeed;
                robot.drive(x, 0, 0, CAMERA_GEAR, x + " " + y + " " + rot);
                robot.stopRandP();
            } else {
                robot.pushGear();
                edu.wpi.first.wpilibj.Timer.delay(0.5);
                robot.retractPiston();
                return;
            }
        }
    }

    @Override
    public void goForwardAndAlign() {
        final Robot robot = this.getRobot();
        while (robot.isEnabled() && robot.isAutonomous()) {
            iterationInit();
            final CameraData cameraData = this.getCurrentCameraGear();
            final LidarData leftLidarData = this.getCurrentLeftLidar();
            final LidarData rightLidarData = this.getCurrentRightLidar();

            if (cameraData == null) {
                robot.rumble(1);
                robot.logMsg(CAMERA_GEAR, "Null camera data");
                robot.stopRandP();
                return;
            }

            if (leftLidarData == null) {
                robot.rumble(1);
                robot.logMsg(LIDAR_GEAR, "Null left lidar data");
                robot.stopRandP();
                return;
            }

            if (rightLidarData == null) {
                robot.rumble(1);
                robot.logMsg(LIDAR_GEAR, "Null right lidar data");
                robot.stopRandP();
                return;
            }

            final int lVal = leftLidarData.getValOnce();
            final int rVal = rightLidarData.getValOnce();
            final int xVal = cameraData.getValOnce();
            final int wVal = cameraData.getWidth();
            System.out.println("right lidar: " + rVal + ", left lidar: " + lVal);

            if (xVal == -1) {
                robot.logMsg(CAMERA_GEAR, "No camera data");
            } else if (xVal < ((wVal / 2 - 30) - wVal * 0.075)) {
                x = xSpeed;
            } else if (xVal > ((wVal / 2 - 30) + wVal * 0.075)) {
                x = -xSpeed;
            } else {
                if (x != 0)
                    x = 0;
            }

            if (lVal > rVal + 25) {
                if (rot != rotSpeed)
                    rot = rotSpeed;
                if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -ySpeed)
                    y = -ySpeed;
                else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != ySpeed)
                    y = ySpeed;
                else {
                    if (y != 0)
                        y = 0;
                }
            } else if (lVal < rVal - 25) {
                if (rot != -rotSpeed)
                    rot = -rotSpeed;
                if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -ySpeed)
                    y = -ySpeed;
                else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != ySpeed)
                    y = ySpeed;
                else {
                    if (y != 0)
                        y = 0;
                }
            } else {
                if (rot != 0)
                    rot = 0;
                if (lVal > upperLidarThreshold && rVal > upperLidarThreshold && y != -ySpeed)
                    y = -ySpeed;
                else if (lVal < lowerLidarThreshold && rVal < lowerLidarThreshold && y != ySpeed)
                    y = ySpeed;
                else {
                    if (y != 0)
                        y = 0;
                }
            }
            robot.drive(x, y, rot, CAMERA_GEAR, x + " " + y + " " + rot);
        }
    }

    @Override
    public void backupByMillis(long time) {
        iterationInit();
        final Robot robot = this.getRobot();
        final HeadingData headingData = this.getCurrentHeading();
        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }
        final long startTime = System.currentTimeMillis();
        long currentTime = System.currentTimeMillis();
        while (robot.isEnabled() && robot.isAutonomous() && (currentTime - startTime) < time) {
            iterationInit();
            final double degrees = headingData.getDegreesOnce();
            // This will be set the first time through
            if (this.getHeadingError() == null)
                this.setHeadingError(new HeadingError(degrees));

            final double errorDegrees = this.getHeadingError().getError(degrees);

            final double turnSpeed;
            final String command;

            if (errorDegrees > THRESHHOLD_DEGREES) {
                // veered right, turn left, turnSpeed will be no less than -1
                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                command = "Forward and counter-clockwise";

            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                // veered left, turn right, turnSpeed will be no more 1
                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                command = "Forward and clockwise";
            } else {
                // On course, drive straight.
                turnSpeed = 0;
                command = "Forward";
            }
            robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
            robot.drive(0, 0.5, turnSpeed, HEADING, command);
            edu.wpi.first.wpilibj.Timer.delay(0.005);
            currentTime = System.currentTimeMillis();
        }
    }

    @Override
    public void forwardByMillis(long time) {
        iterationInit();
        final Robot robot = this.getRobot();
        final HeadingData headingData = this.getCurrentHeading();
        if (headingData == null) {
            robot.logMsg(HEADING, "Null heading data");
            return;
        }
        final long startTime = System.currentTimeMillis();
        while (robot.isEnabled() && robot.isAutonomous() && (System.currentTimeMillis() - startTime) < time) {
            iterationInit();
            final double degrees = headingData.getDegreesOnce();
            // This will be set the first time through
            if (this.getHeadingError() == null)
                this.setHeadingError(new HeadingError(degrees));

            final double errorDegrees = this.getHeadingError().getError(degrees);

            final double turnSpeed;
            final String command;

            if (errorDegrees > THRESHHOLD_DEGREES) {
                // veered right, turn left, turnSpeed will be no less than -1
                turnSpeed = Math.max(-errorDegrees * PID_CORRECTION_STRAIGHT, -0.2);
                command = "Forward and counter-clockwise";

            } else if (errorDegrees < (THRESHHOLD_DEGREES * -1)) {
                // veered left, turn right, turnSpeed will be no more 1
                turnSpeed = Math.min(-errorDegrees * PID_CORRECTION_STRAIGHT, 0.2);
                command = "Forward and clockwise";
            } else {
                // On course, drive straight.
                turnSpeed = 0;
                command = "Forward";
            }
            robot.logMsg(HEADING, "error: " + errorDegrees + " turn speed: " + turnSpeed);
            robot.drive(0, -0.75, 0, HEADING, command);
            edu.wpi.first.wpilibj.Timer.delay(0.005);
        }
    }

    @Override
    public void stop() {
        iterationInit();
        final Robot robot = this.getRobot();
        robot.drive(0, 0, 0, HEADING, "stop");
    }

}
