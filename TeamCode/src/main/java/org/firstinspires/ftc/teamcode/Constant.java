package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constant {
    public enum Alliance {RED, BLUE};
    public static class VISION {
        public static final String CAMERA_NAME = "Webcam 1";
        public static final Size CAMERA_RESOLUTION = new Size(320, 240);
    }

    public static class ARM {
        public static final double ANGLE_TOLERANCE = 1;
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

    public static class KALMAN_GAIN {
        public static int N = 3; // number of estimate
        public static class POTENTIOMETER {
            public static double Q = 1; // put more on sensor
            public static double R = 0; // put more on regression
        }
    }
}
