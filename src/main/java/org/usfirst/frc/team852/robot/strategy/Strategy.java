package org.usfirst.frc.team852.robot.strategy;

import org.usfirst.frc.team852.robot.Robot;

public interface Strategy {

    void reset();
    
    void xboxAButtonPressed(Robot robot);

    void xboxBButtonPressed(Robot robot);

    void xboxXButtonPressed(Robot robot);

    void xboxYButtonPressed(Robot robot);

    void xboxLBButtonPressed(Robot robot);

    void xboxRBButtonPressed(Robot robot);

    void xboxStartButtonPressed(Robot robot);
}
