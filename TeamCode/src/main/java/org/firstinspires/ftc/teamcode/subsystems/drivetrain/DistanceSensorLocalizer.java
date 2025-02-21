package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.DoubleSupplier;

public class DistanceSensorLocalizer {
    private DistanceSensor leftDis, rightDis, rearDis;
    private final DistanceUnit Unit = DistanceUnit.CM;
    private LinearOpMode opMode;

    private DoubleSupplier leftDisVal = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return leftDis.getDistance(Unit);
        }
    };
    private Estimator leftDisFilter = new KalmanEstimator(leftDisVal, 1, 0, 3);
    private DoubleSupplier rightDisVal = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return rightDis.getDistance(Unit);
        }
    };
    private Estimator rightDisFilter = new KalmanEstimator(rightDisVal, 1, 0, 3);
    private DoubleSupplier rearDisVal = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return rearDis.getDistance(Unit);
        }
    };
    private Estimator rearDisFilter = new KalmanEstimator(rearDisVal, 1, 0, 3);


    public DistanceSensorLocalizer(LinearOpMode _opMode) {
        this.opMode = _opMode;
    }
    public void init() {
        leftDis = opMode.hardwareMap.get(DistanceSensor.class, "leftDis");
        rightDis = opMode.hardwareMap.get(DistanceSensor.class, "rightDis");
        rearDis = opMode.hardwareMap.get(DistanceSensor.class, "rearDis");
    }

    public double getLeftDis() { return leftDisFilter.update(); }
    public double getRightDis() { return rightDisFilter.update(); }
    public double getRearDis() { return rearDisFilter.update(); }
}
