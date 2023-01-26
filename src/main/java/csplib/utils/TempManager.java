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

    private TempManager() {

    }

    public static void addMotor(CSP_Motor... motors) {
        for (CSP_Motor motor : motors) {
            motorsList.add(motor);
        }
    }

    public static void monitor() {
        for (CSP_Motor motor : motorsList) {
            if (motor.getTemperature() > Constants.robot.MAX_TEMP) {
                DriverStation.reportWarning("Motor " + motor.getID() + " Critical Temperature Reached", false);
            }
        }
    }
}
