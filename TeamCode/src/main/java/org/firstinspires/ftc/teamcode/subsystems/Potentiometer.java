package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constant.*;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class Potentiometer {
    private final AnalogInput potentiometer;
    private final KalmanFilter filter = new KalmanFilter(KALMAN_GAIN.POTENTIOMETER.Q, KALMAN_GAIN.POTENTIOMETER.R, KALMAN_GAIN.N);
    public Potentiometer (LinearOpMode opMode) {
        potentiometer = opMode.hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    /**
     * get raw angle from potentiometer
     * @return double angle in range [0, 270]
     */
    public double getRawAngle() {
        return potentiometer.getVoltage() * 81.8;
    }

    /**
     * get filtered angle from potentiometer
     * @return double angle in range [0, 270]
     */
    public double getFilteredAngle() {
        return filter.estimate(getRawAngle());
    }
}
