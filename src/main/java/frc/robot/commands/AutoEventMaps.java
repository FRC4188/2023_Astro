// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class AutoEventMaps {
  public Map<String, Command> b1Map = Map.ofEntries(entry("Command", new SequentialCommandGroup()));
}
