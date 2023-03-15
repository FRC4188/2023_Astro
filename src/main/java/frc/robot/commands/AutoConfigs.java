// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.claw.Outtake;
import frc.robot.commands.groups.Reset;
import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class AutoConfigs {
  public static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(Constants.drivetrain.MAX_VELOCITY, Constants.drivetrain.MAX_ACCEL);

  public static final HashMap<String, Command> EVENTS =
      new HashMap<>(
          Map.ofEntries(
              Map.entry("Set High Cone", new SetPosition(Constants.arm.configs.HIGH)),
              Map.entry("Outtake", new Outtake()),
              Map.entry("Reset", new Reset()),
              Map.entry("Set Intake Cube", new SetPosition(Constants.arm.configs.FLOOR_CUBE))));

  public static final class Test {
    public static final PathConstraints[] CONSTRAINTS = {
      new PathConstraints(4, 2), new PathConstraints(0.5, 0.2)
    };
  }
}
