// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.shoulder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Shoulder;

public class HoldShoulder extends CommandBase {
  private Shoulder shoulder = Shoulder.getInstance();

  private double setAngle;

  /** Creates a new HoldShoulder. */
  public HoldShoulder() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setAngle = shoulder.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setAngle(setAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
