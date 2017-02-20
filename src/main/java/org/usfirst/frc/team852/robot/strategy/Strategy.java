package org.usfirst.frc.team852.robot.strategy;

import org.usfirst.frc.team852.robot.Robot;
import org.usfirst.frc.team852.robot.data.CameraData;
import org.usfirst.frc.team852.robot.data.HeadingData;
import org.usfirst.frc.team852.robot.data.LidarData;

public interface Strategy {

    void reset();

    void resetHeading();

    void iterationInit(Robot robot);

    void onXboxA(Robot robot);

    void onXboxB(Robot robot);

    void onXboxX(Robot robot);

    void onXboxY(Robot robot);

    void onXboxLB(Robot robot);

    void onXboxRB(Robot robot);

    void onXboxBack(Robot robot);

    void onXboxStart(Robot robot);

    void onXboxLS(Robot robot);

    void onXboxRS(Robot robot);

    CameraData getCurrentCameraGear();

    LidarData getCurrentLeftLidar();

    LidarData getCurrentRightLidar();

    LidarData getCurrentRearLidar();

    LidarData getCurrentFrontLidar();

    HeadingData getCurrentHeading();
}
