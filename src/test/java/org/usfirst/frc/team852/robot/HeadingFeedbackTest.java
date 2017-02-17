package org.usfirst.frc.team852.robot;

import org.assertj.core.data.Offset;
import org.junit.Test;
import org.usfirst.frc.team852.robot.navigation.HeadingFeedback;

import static org.assertj.core.api.Assertions.assertThat;

public class HeadingFeedbackTest {

    private static final Offset<Double> offset = Offset.offset(0.001);

    @Test
    public void lessThan90Test() {
        final HeadingFeedback fb = new HeadingFeedback(15);
        assertThat(fb.getError(15)).isEqualTo(0);
        assertThat(fb.getError(10)).isEqualTo(-5);
        assertThat(fb.getError(20)).isEqualTo(5);

        assertThat(fb.getError(360)).isEqualTo(-15);
        assertThat(fb.getError(359.9)).isCloseTo(-15.1, offset);
        assertThat(fb.getError(359.7)).isCloseTo(-15.3, offset);
        assertThat(fb.getError(355)).isEqualTo(-20);
        assertThat(fb.getError(270)).isEqualTo(-105);

        assertThat(fb.getError(90)).isEqualTo(75);
        assertThat(fb.getError(180)).isEqualTo(165);
        assertThat(fb.getError(190)).isEqualTo(175);

        assertThat(fb.getError(195)).isEqualTo(180);
    }

    @Test
    public void greaterThan270Test() {
        final HeadingFeedback fb = new HeadingFeedback(345);

        assertThat(fb.getError(345)).isEqualTo(0);
        assertThat(fb.getError(340)).isEqualTo(-5);
        assertThat(fb.getError(350)).isEqualTo(5);

        assertThat(fb.getError(10)).isEqualTo(25);
        assertThat(fb.getError(10.9)).isCloseTo(25.9, offset);
        assertThat(fb.getError(15.1)).isCloseTo(30.1, offset);
        assertThat(fb.getError(90)).isEqualTo(105);
        assertThat(fb.getError(110)).isEqualTo(125);

        assertThat(fb.getError(120)).isEqualTo(135);

        assertThat(fb.getError(270)).isEqualTo(-75);
        assertThat(fb.getError(180)).isEqualTo(-165);
        assertThat(fb.getError(170)).isEqualTo(-175);

        assertThat(fb.getError(165)).isEqualTo(180);
    }

    @Test
    public void between90And270Test() {
        final HeadingFeedback fb = new HeadingFeedback(180);

        assertThat(fb.getError(180)).isEqualTo(0);
        assertThat(fb.getError(170)).isEqualTo(-10);
        assertThat(fb.getError(190)).isEqualTo(10);

        assertThat(fb.getError(180.1)).isCloseTo(.1, offset);
        assertThat(fb.getError(179.9)).isCloseTo(-.1, offset);
        assertThat(fb.getError(190.1)).isCloseTo(10.1, offset);
        assertThat(fb.getError(169.9)).isCloseTo(-10.1, offset);

        assertThat(fb.getError(10)).isEqualTo(-170);
        assertThat(fb.getError(350)).isEqualTo(170);
    }

    @Test
    public void at0() {
        final HeadingFeedback fb = new HeadingFeedback(0);

        assertThat(fb.getError(0)).isEqualTo(0);
        assertThat(fb.getError(360)).isEqualTo(0);
        assertThat(fb.getError(180)).isEqualTo(180);
        assertThat(fb.getError(10)).isEqualTo(10);
        assertThat(fb.getError(190)).isEqualTo(-170);

        assertThat(fb.getError(.1)).isCloseTo(.1, offset);
        assertThat(fb.getError(359.9)).isCloseTo(-.1, offset);
    }

    @Test
    public void at90() {
        final HeadingFeedback fb = new HeadingFeedback(90);

        assertThat(fb.getError(90)).isEqualTo(0);
        assertThat(fb.getError(180)).isEqualTo(90);
        assertThat(fb.getError(270)).isEqualTo(180);
        assertThat(fb.getError(360)).isEqualTo(-90);
        assertThat(fb.getError(0)).isEqualTo(-90);
    }

    @Test
    public void at180() {
        final HeadingFeedback fb = new HeadingFeedback(180);

        assertThat(fb.getError(180)).isEqualTo(0);
        assertThat(fb.getError(0)).isEqualTo(180);
        assertThat(fb.getError(360)).isEqualTo(180);
        assertThat(fb.getError(10)).isEqualTo(-170);
        assertThat(fb.getError(-10)).isEqualTo(170);
    }

    @Test
    public void at270() {
        final HeadingFeedback fb = new HeadingFeedback(270);

        assertThat(fb.getError(270)).isEqualTo(0);
        assertThat(fb.getError(360)).isEqualTo(90);
        assertThat(fb.getError(0)).isEqualTo(90);
        assertThat(fb.getError(90)).isEqualTo(180);
        assertThat(fb.getError(180)).isEqualTo(-90);

        assertThat(fb.getError(180.1)).isCloseTo(-89.9, offset);
        assertThat(fb.getError(179.9)).isCloseTo(-90.1, offset);
    }
}
