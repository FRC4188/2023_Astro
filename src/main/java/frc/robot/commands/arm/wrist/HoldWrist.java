// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.arm.wrist;
import frc.robot.subsystems.arm.Wrist;
import java.util.function.DoubleSupplier;

public class HoldWrist extends CommandBase {
  private Wrist wrist = Wrist.getInstance();

  private double lastAngle;
  private DoubleSupplier set;
  private double wristSet;
  private double prevSet;
  /** Creates a new HoldWrist. */
  public HoldWrist(DoubleSupplier set) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    this.set = set;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastAngle = wrist.getMotorAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setAngle(wristSet);
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
