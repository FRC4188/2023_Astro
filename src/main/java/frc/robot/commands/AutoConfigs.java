// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.claw.Intake;
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
              Map.entry("Set Mid Cone", new SetPosition(Constants.arm.configs.MID_CONE)),
              Map.entry("Set Low Cone", new SetPosition(Constants.arm.configs.LOW_CONE)),
              Map.entry(
                  "Outtake",
                  new SequentialCommandGroup(new WaitCommand(1.0), new Outtake().withTimeout(0.5))),
              Map.entry("Reset", new Reset()),
              Map.entry("Set Cube", new SetPosition(Constants.arm.configs.FLOOR_CUBE)),
              Map.entry("Set Up Cone", new SetPosition(Constants.arm.configs.FLOOR_CONE)),
              Map.entry("Intake", new Intake().withTimeout(0.5))));

  public static final class three2P {
    public static final PathConstraints[] CONSTRAINTS = {new PathConstraints(5, 3)};
  }

  public static final class three1P {
    public static final PathConstraints[] CONSTRAINTS = {
      new PathConstraints(5, 3), new PathConstraints(2, 2), new PathConstraints(5, 3)
    };
  }
}
