package org.usfirst.frc.team852.robot.navigation;

public class HeadingFeedback {

    private final double initialHeading;

    public HeadingFeedback(final double initialHeading) {
        this.initialHeading = initialHeading;
    }

    public double getInitialHeading() {
        return this.initialHeading;
    }

    public double getError(final double heading) {
        final double correction;
        if (this.initialHeading <= 120)
            correction = heading - (heading > (240) ? (this.initialHeading + 360) : this.initialHeading);
        else if (this.initialHeading >= 240)
            correction = (heading < (120) ? (heading + 360) : heading) - this.initialHeading;
        else
            correction = heading - this.initialHeading;

        if (correction <= -180)
            return correction + 360;
        else if (correction > 180)
            return correction - 360;
        return correction;
    }
}
