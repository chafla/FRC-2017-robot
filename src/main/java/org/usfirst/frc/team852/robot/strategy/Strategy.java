package org.usfirst.frc.team852.robot.strategy;

import org.usfirst.frc.team852.robot.Robot;

public interface Strategy {

    void reset();

    void resetHeading();

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
}
