// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.groups.ScoreHigh;

/** Add your docs here. */
public class AutoEventMaps {
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(Constants.drivetrain.MAX_VELOCITY, Constants.drivetrain.MAX_ACCEL);

    public static final class Test {
        public static final HashMap<String, Command> EVENTS = new HashMap<>(
            Map.ofEntries(
                Map.entry("Score Cube", new ScoreHigh())
            ));
        
        public static final PathConstraints[] CONSTRAINTS = {
            new PathConstraints(4, 2),
            new PathConstraints(0.5, 0.2)
        };
    }



}
