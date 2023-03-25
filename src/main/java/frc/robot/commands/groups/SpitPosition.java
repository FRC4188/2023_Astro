// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.claw.Outtake;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitPosition extends SequentialCommandGroup {

  /** Creates a new SpitPosition. */
  public SpitPosition(double[] cube, double[] cone) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SetPosition(cube, cone)
            .until(() -> (Wrist.getInstance().atGoal() && Telescope.getInstance().atGoal())),
        new Outtake());
  }
}
