// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.drivetrain.angle;
import frc.robot.subsystems.arm.Shoulder;

public class SetShoulderAngle extends CommandBase {
  private Shoulder shoulder = Shoulder.getInstance();

  private double angle;
  /** Creates a new SetShoulderAngle. */
  public SetShoulderAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulder.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
