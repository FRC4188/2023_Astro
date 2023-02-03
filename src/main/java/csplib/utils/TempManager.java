// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package csplib.utils;

import java.util.ArrayList;

import csplib.motors.CSP_Motor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Add your docs here. */
public final class TempManager {
    private static ArrayList<CSP_Motor> motorsList = new ArrayList<CSP_Motor>();

    /**
     * Add motors to be monitored
     * @param motors motors to be monitored
     */
    public static void addMotor(CSP_Motor... motors) {
        for (CSP_Motor motor : motors) {
            motorsList.add(motor);
        }
    }

    /**
     * Report warning if temperature > 50, report error if temperature > 60
     */
    public static void monitor() {
        for (CSP_Motor motor : motorsList) {
            if (motor.getTemperature() > Constants.robot.MAX_TEMP) {
                DriverStation.reportWarning("Motor " + motor.getID() + " Max Temperature Reached", false);
            } else if (motor.getTemperature() > Constants.robot.MAX_TEMP + 10) {
                DriverStation.reportError("Motor " + motor.getID() + " Critical Temperature Reached", false);
            }
        }
    }
}
