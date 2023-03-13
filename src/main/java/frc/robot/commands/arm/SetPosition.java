// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.shoulder.SetShoulderAngle;
import frc.robot.commands.arm.telescope.SetTelescopePosition;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.commands.arm.wrist.SetWristAngle;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.arm.Telescope;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPosition extends SequentialCommandGroup {
  /** Creates a new SetPosition. */
  public SetPosition(double shoulderAngle, double telescopeLength, double wristAngle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ZeroTelescope(),
                new SetShoulderAngle(shoulderAngle)
                    .until(() -> Math.abs(shoulderAngle - Shoulder.getInstance().getAngle()) < 1),
                new ParallelCommandGroup(
                    new SetTelescopePosition(telescopeLength)
                        .unless(
                            () ->
                                Math.abs(telescopeLength - Telescope.getInstance().getPosition())
                                    < 0.15),
                    new SetShoulderAngle(shoulderAngle))),
            new SetWristAngle(wristAngle)));
  }
}
