// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.commands.claw.Outtake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Toss extends SequentialCommandGroup {
  /** Creates a new Toss. */
  public Toss() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ZeroTelescope(),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(new WaitCommand(0.7), new Outtake().withTimeout(0.5)),
            new SetPosition(0, 1, 0).withTimeout(2.0)));
  }
}
