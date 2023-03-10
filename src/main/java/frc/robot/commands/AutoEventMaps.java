// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class AutoEventMaps {
  public static HashMap<String, Command> b1Map =
      new HashMap<>(Map.ofEntries(Map.entry("Command", new SequentialCommandGroup())));
}
