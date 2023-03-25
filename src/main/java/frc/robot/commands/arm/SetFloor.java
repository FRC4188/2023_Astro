// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.shoulder.SetShoulderAngle;
import frc.robot.commands.arm.telescope.SetTelescopePosition;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.commands.arm.wrist.SetWristAngle;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.claw.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFloor extends SequentialCommandGroup {
  private Claw claw = Claw.getInstance();
  private Shoulder shoulder = Shoulder.getInstance();

  /** Creates a new SetFloorCube. */
  private SetFloor(double shoulderAngle, double telescopeLength, double wristAngle) {
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new ZeroTelescope(),
          new SetTelescopePosition(telescopeLength),
          new SetShoulderAngle(shoulderAngle)
        ),
        new SetWristAngle(wristAngle)
      )
    );
  }

  public SetFloor(double[] cube, double[] cone) {
    double[] ffc = Constants.arm.configs.FLIPPED_FLOOR_CUBE;
    addCommands(
      new ConditionalCommand(
          new ConditionalCommand(
              new SetFloor(ffc[0], ffc[1], ffc[2]), 
              new SetFloor(-cone[0], cone[1], -cone[2]), 
              claw::getIsCube),
          new ConditionalCommand(
              new SetFloor(cube[0], cube[1], cube[2]), 
              new SetFloor(cone[0], cone[1], cone[2]), 
              claw::getIsCube),
          shoulder::getIsFlipped
        )
    );
  }
}
