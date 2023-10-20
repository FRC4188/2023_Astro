// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetCube;
import frc.robot.commands.arm.SetFlip;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.arm.shoulder.SetShoulderAngle;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.commands.claw.Intake;
import frc.robot.commands.claw.Outtake;
import frc.robot.commands.drive.Balance;
import frc.robot.commands.groups.Reset;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.Drivetrain;
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
                  "Set High",
                  new ConditionalCommand(
                      new SetPosition(
                              Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE)
                          .withTimeout(1.3),
                      new SetPosition(
                              Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE)
                          .withTimeout(1.8),
                      Claw.getInstance()::getIsCube)),
              Map.entry(
                  "Set High Delay",
                  new ConditionalCommand(
                      new SetPosition(
                              Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE)
                          .withTimeout(2.0),
                      new SetPosition(
                              Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE)
                          .withTimeout(2.5),
                      Claw.getInstance()::getIsCube)),
              Map.entry(
                  "Set Mid",
                  new ConditionalCommand(
                      new SetPosition(
                              Constants.arm.configs.MID_CUBE, Constants.arm.configs.MID_CONE)
                          .withTimeout(1.3),
                      new SetPosition(
                              Constants.arm.configs.MID_CUBE, Constants.arm.configs.MID_CONE)
                          .withTimeout(1.8),
                      Claw.getInstance()::getIsCube)),
              Map.entry(
                  "Spit Low",
                  new ConditionalCommand(
                      new SetPosition(
                              Constants.arm.configs.LOW_CUBE, Constants.arm.configs.LOW_CONE)
                          .withTimeout(1.4)
                          .andThen(new Outtake().withTimeout(0.2)),
                      new SetPosition(
                              Constants.arm.configs.LOW_CUBE, Constants.arm.configs.LOW_CONE)
                          .withTimeout(1.4)
                          .andThen(new Outtake().withTimeout(0.2)),
                      Claw.getInstance()::getIsCube)),
              Map.entry(
                  "Set Floor",
                  new SetPosition(
                      Constants.arm.configs.FLOOR_CUBE, Constants.arm.configs.FLOOR_CONE)),
              Map.entry("Prep Shoulder", new SetShoulderAngle(50)), // just in the middle somwhere
              Map.entry("Set Cube", new SetCube()),
              Map.entry(
                  "Stop Drivetrain",
                  new InstantCommand(
                      () -> Drivetrain.getInstance().zeroPower(), Drivetrain.getInstance())),
              Map.entry("Zero Telescope", new ZeroTelescope()),
              Map.entry("Reset Flipped", new SetFlip().andThen(new Reset())),
              Map.entry("Set Flipped", new SetFlip()),
              Map.entry("Reset", new Reset()),
              Map.entry("Print", new PrintCommand("IT DOESNT END")),
              Map.entry("Intake", new Intake()),
              Map.entry("Outtake", new Outtake().withTimeout(0.2)),
              Map.entry("Balance", new Balance()),
              Map.entry("Timed Balance", new Balance().withTimeout(3.0)),
              Map.entry(
                  "Reset Balance",
                  new SetFlip().andThen(new Reset()).withTimeout(1.5).andThen(new Balance()))));

  public static final class RFlat2 {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(5, 3)};
  }

  public static final class RFlat3 {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(7, 3.5)};
  }

  public static final class BFlat3 {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(7, 3.5)};
  }

  public static final class RMid15P {
    public static final PathConstraints[] CONSTRAINTS = {
      new PathConstraints(1, 2), new PathConstraints(4, 3), new PathConstraints(1.5, 2)
    };
  }



  public static final class RMid2P {
    public static final PathConstraints[] CONSTRAINTS = {
      new PathConstraints(3, 2),
      new PathConstraints(6, 4),
      new PathConstraints(4, 3),
      new PathConstraints(2, 1),
      new PathConstraints(5, 3)
    };
  }

  public static final class PerfectAuto {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(3, 1)};
  }
}
