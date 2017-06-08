package org.usfirst.frc.team852.robot.navigation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Modified version of PIDControl that incorporates helper methods for working with the smartdashboard.
 * We can use the smartdashboard to publish and update values.
 */

public class PIDControlDashboard extends PIDControl {

    private String name;

    public PIDControlDashboard(double p, double i, double d, String name) {

        super(p, i, d);
        this.name = name;

    }

    public PIDControlDashboard(String name) {
        super();
        this.name = name;
    }

    // Publish current PID values to smartdashboard
    public void publish() {

        SmartDashboard.putNumber(String.format("%s P", this.name), this.getpGain());
        SmartDashboard.putNumber(String.format("%s I", this.name), this.getiGain());
        SmartDashboard.putNumber(String.format("%s D", this.name), this.getdGain());

    }

    // Update PID values from the smart dashboard
    public void updateFromDash() {

        this.setP(SmartDashboard.getNumber(String.format("%s P", this.name), this.getpGain()));
        this.setI(SmartDashboard.getNumber(String.format("%s I", this.name), this.getiGain()));
        this.setD(SmartDashboard.getNumber(String.format("%s D", this.name), this.getdGain()));

    }




}
