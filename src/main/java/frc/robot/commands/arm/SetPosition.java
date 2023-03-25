// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.shoulder.SetShoulderAngle;
import frc.robot.commands.arm.telescope.SetTelescopePosition;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.commands.arm.wrist.SetWristAngle;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.claw.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetPosition extends SequentialCommandGroup {
  private Shoulder shoulder = Shoulder.getInstance();
  private Claw claw = Claw.getInstance();

  /** Creates a new SetPosition. */
  private SetPosition(double shoulderAngle, double telescopeLength, double wristAngle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ZeroTelescope(),
                new SetShoulderAngle(shoulderAngle)
                    .until(() -> Shoulder.getInstance().atGoal(shoulderAngle)),
                new ParallelCommandGroup(
                    new SetShoulderAngle(shoulderAngle),
                    new SetTelescopePosition(telescopeLength))),
            new SetWristAngle(wristAngle)));
  }

  public SetPosition(double[] cube, double[] cone) {
    addCommands(
        new ConditionalCommand(
            new ConditionalCommand(
                new SetPosition(-cube[0], cube[1], -cube[2]),
                new SetPosition(-cone[0], cone[1], -cone[2]),
                claw::getIsCube),
            new ConditionalCommand(
                new SetPosition(cube[0], cube[1], cube[2]),
                new SetPosition(cone[0], cone[1], cone[2]),
                claw::getIsCube),
            shoulder::getIsFlipped));
  }
}
