package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;

@Config
public class Constant {
    public enum Alliance {RED, BLUE}

    ;

    public static class VISION {
        public static final String CAMERA_NAME = "Webcam 1";
        public static final Size CAMERA_RESOLUTION = new Size(320, 240);
    }

    public static class CLAW {
        public static final double GRAB = 1;
        public static final double RELEASE = 0;
    }

    public static class VER_SLIDE {
        public static final double RETRACT = 7;
        public static final double EXTEND = 2700;
        public static final double TOLERANCE = 1;
        public static PIDCoefficientsEx SLIDE_PID_COEFFICIENTS = new PIDCoefficientsEx(
                0.0125, 0.7, 0.008, 0, 0, 0
        );
        public static FeedforwardCoefficientsEx SLIDE_FEEDFORWARD_COEFFICIENTS = new FeedforwardCoefficientsEx(
                0, 0, 0, 0.1, 0.0
        );

        public static class KALMAN_GAIN {
            public static int N = 3; // number of estimate
            public static double Q = 1; // put more on sensor
            public static double R = 0; // put more on regression
        }
    }

    public static class SPEED {
        public static final double STRAIGHT_SPEED = 0.8;
        public static final double STRAFE_SPEED = 0.8;
        public static final double TURN_SPEED = 0.8;
    }

    public static class GAMEPAD_SENSITIVITY {
        // Joystick sensitivity
        public static final double SENSE_X = 0.15;
        public static final double SENSE_Y = 0.15;
        public static final double SENSE_Z = 0.15;

        // Analog buttons
        public static final double SENSE_TRIGGER = 0.25;
    }

    public static class HOR_SLIDE {
        public static final double MAX_POS = 0.38;
        public static final double WRIST_EXTEND_POS = 0;
        public static final double WRIST_RETRACT_POS = 1;
    }
}
