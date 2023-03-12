// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.commands.arm.wrist.SetWristAngle;
import frc.robot.subsystems.arm.Shoulder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends SequentialCommandGroup {
  private Shoulder shoulder = Shoulder.getInstance();
  /** Creates a new Stow. */
  public Stow() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ZeroTelescope(),
      new RunCommand(() -> shoulder.setAngle(0), shoulder),
      new SetWristAngle(Constants.arm.wrist.LOWER_LIMIT + 10)
    );
  }
}
