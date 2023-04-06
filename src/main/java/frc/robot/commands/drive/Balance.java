// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.sensors.Sensors;

public class Balance extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Sensors sensors = Sensors.getInstance();

  /** Creates a new Balance. */
  public Balance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (sensors.getPitch() > 10) drivetrain.setVelocity(-0.25);
    else if (sensors.getPitch() < -10) drivetrain.setVelocity(0.25);
    else drivetrain.setVelocity(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
