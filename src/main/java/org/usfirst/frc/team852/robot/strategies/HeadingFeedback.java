package org.usfirst.frc.team852.robot.strategies;

public class HeadingFeedback {

    private final double initialHeading;

    public HeadingFeedback(final double initialHeading) {
        this.initialHeading = initialHeading;
    }

    public double getInitialHeading() {
        return this.initialHeading;
    }

    public double getError(final double recentVal) {
        // Assume that we are no more than 90 degrees off line
        if (this.initialHeading <= 90 + 45)
            return recentVal - (recentVal > (180 + 45) ? (this.initialHeading + 360) : this.initialHeading);
        else if (this.initialHeading >= 270 - 45)
            return (recentVal < (180 - 45) ? (recentVal + 360) : recentVal) - this.initialHeading;
        else
            return recentVal - this.initialHeading;
    }
}
