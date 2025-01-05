package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Constant {
    public enum Alliance {RED, BLUE};
    public static class VISION {
        public static final String CAMERA_NAME = "Webcam 1";
        public static final Size CAMERA_RESOLUTION = new Size(320, 240);
    }

    public static class ARM {
        public static final double TICK_PER_REV = 288;
        public static final double GEAR_RATIO = 4;
        public static final double DEG_PER_TICK = 360 / (TICK_PER_REV * GEAR_RATIO);
        public static final double TICK_OFFSET = 0;
        public static final double ANGLE_TOLERANCE = 1;
        public static PIDCoefficientsEx ARM_PID_COEFFICIENTS = new PIDCoefficientsEx(
                0, 0, 0, 0, 0, 0
        );
        public static FeedforwardCoefficientsEx ARM_FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficientsEx(
                0, 0, 0, 0, 0
        );
        public static WPILibMotionProfile.Constraints ARM_VA_CONSTRAINT =
                new WPILibMotionProfile.Constraints(
                        200, 150
                );

        public static class KALMAN_GAIN {
            public static int N = 3; // number of estimate
            public static double Q = 1; // put more on sensor
            public static double R = 0; // put more on regression
        }
    }

    public static class SLIDE {
        public static final double TOLERANCE = 1;
        public static PIDCoefficientsEx SLIDE_PID_COEFFICIENTS = new PIDCoefficientsEx(
                0, 0, 0, 0, 0, 0
        );
        public static FeedforwardCoefficientsEx SLIDE_FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficientsEx(
                0, 0, 0, 0, 0
        );
        public static WPILibMotionProfile.Constraints SLIDE_VA_CONSTRAINT =
                new WPILibMotionProfile.Constraints(
                        200, 150
                );

        public static class KALMAN_GAIN {
            public static int N = 3; // number of estimate
            public static double Q = 1; // put more on sensor
            public static double R = 0; // put more on regression
        }
    }

    public static class SPEED {
        public static final double STRAIGHT_SPEED = 1;
        public static final double STRAFE_SPEED = 1;
        public static final double TURN_SPEED = 1;
    }

    public static class GAMEPAD_SENSITIVITY {
        // Joystick sensitivity
        public static final double SENSE_X = 0.15;
        public static final double SENSE_Y = 0.15;
        public static final double SENSE_Z = 0.15;

        // Analog buttons
        public static final double SENSE_TRIGGER = 0.25;
    }
}
