package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Utilities functions, wrapper, etc...
 */
public class Utility {
    /**
     * Enable bulk read for all hubs
     *
     * @param hardwareMap hardwareMap object from opMode
     */
    public static List<LynxModule> enableBulkRead(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        return allHubs;
    }

    /**
     * Apply sense to input
     *
     * @param input     input value
     * @param threshold sense threshold
     * @return input if abs(input) > threshold, else 0
     */
    public static double sense(double input, double threshold) {
        if (Math.abs(input) >= threshold) {
            return input;
        } else {
            return 0;
        }
    }
}