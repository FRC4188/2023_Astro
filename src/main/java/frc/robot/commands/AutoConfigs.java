// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetCube;
import frc.robot.commands.arm.SetFlip;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.arm.drive.Balance;
import frc.robot.commands.claw.Intake;
import frc.robot.commands.claw.Outtake;
import frc.robot.commands.groups.Reset;
import frc.robot.commands.groups.SpitPosition;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.claw.Claw;

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
                  new ParallelDeadlineGroup(
                      new SequentialCommandGroup(
                          new WaitCommand(2.0), 
                          new Outtake().withTimeout(0.5)),
                      new SetPosition(Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE))),
              Map.entry(
                  "Spit High Cube",
                  new ParallelDeadlineGroup(
                      new SequentialCommandGroup(
                          new WaitCommand(2.0), new Intake().withTimeout(0.5)),
                      new SetPosition(Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CUBE))),
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
                  "Set Floor Cube",
                  new SetPosition(
                      Constants.arm.configs.FLOOR_CUBE, Constants.arm.configs.FLOOR_CUBE)),
              Map.entry("Set Cube", new SetCube()),
              Map.entry("Reset Flipped", 
                new InstantCommand(
                  () -> 
                    Shoulder.getInstance()
                    .setFlip(!Shoulder.getInstance()
                    .getIsFlipped()), Shoulder.getInstance())
                    .andThen(new Reset())),
              Map.entry("Set Flipped", 
                new InstantCommand(
                  () -> 
                    Shoulder.getInstance()
                    .setFlip(!Shoulder.getInstance()
                    .getIsFlipped()), Shoulder.getInstance())),
              Map.entry("Reset", new Reset()),
              Map.entry("Intake", 
                new ConditionalCommand(new Outtake().withTimeout(0.3), new Intake().withTimeout(0.3), Claw.getInstance()::getIsCube)),
              Map.entry("Outtake", 
                new ConditionalCommand(new Intake().withTimeout(0.3), new Outtake().withTimeout(0.3), Claw.getInstance()::getIsCube)),
              Map.entry("Balance",
                new Balance())
            )
        );
            

  public static final class RFlat2 {
    public static final PathConstraints[] CONSTRAINTS = {
      new PathConstraints(5, 3)
    };
  }

  public static final class RMid15P {
    public static final PathConstraints[] CONSTRAINTS = {
      new PathConstraints(3, 1),
      new PathConstraints(4, 2),
      new PathConstraints(4, 2)
    };
  }

  public static final class PerfectAuto {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(3, 1)};
  }

}
