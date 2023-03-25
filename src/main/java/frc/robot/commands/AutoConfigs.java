// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.arm.SetCube;
import frc.robot.commands.arm.SetFlip;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.claw.Intake;
import frc.robot.commands.claw.Outtake;
import frc.robot.commands.groups.Reset;
import frc.robot.commands.groups.SpitPosition;
import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public class AutoConfigs {
  public static final PathConstraints DEFAULT_CONSTRAINTS =
      new PathConstraints(Constants.drivetrain.MAX_VELOCITY, Constants.drivetrain.MAX_ACCEL);

  public static final HashMap<String, Command> EVENTS =
      new HashMap<>(
          Map.ofEntries(
              Map.entry(
                  "Spit High",
                  new SpitPosition(
                      Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE)),
              Map.entry(
                  "Spit Mid",
                  new SpitPosition(Constants.arm.configs.MID_CUBE, Constants.arm.configs.MID_CONE)),
              Map.entry(
                  "Spit Low",
                  new SpitPosition(Constants.arm.configs.LOW_CUBE, Constants.arm.configs.LOW_CONE)),
              Map.entry(
                  "Set High",
                  new SetPosition(
                      Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE)),
              Map.entry(
                  "Set Mid",
                  new SetPosition(Constants.arm.configs.MID_CUBE, Constants.arm.configs.MID_CONE)),
              Map.entry(
                  "Set Low",
                  new SetPosition(Constants.arm.configs.LOW_CUBE, Constants.arm.configs.LOW_CONE)),
              Map.entry(
                  "Set Floor",
                  new SetPosition(
                      Constants.arm.configs.FLOOR_CUBE, Constants.arm.configs.FLOOR_CONE)),
              Map.entry("Set Cube", new SetCube()),
              Map.entry("Set Flipped", new SetFlip()),
              Map.entry("Reset", new Reset()),
              Map.entry("Intake", new Intake().withTimeout(0.5)),
              Map.entry("Outtake", new Outtake().withTimeout(0.5))));

  public static final class three2P {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(5, 3)};
  }

  public static final class three1P {
    public static final PathConstraints[] CONSTRAINTS = {
      new PathConstraints(5, 3), new PathConstraints(2, 2), new PathConstraints(5, 3)
    };
  }

  public static final class PerfectAuto {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(3, 1)};
  }
}
