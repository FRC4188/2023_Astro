// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm.telescope;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Telescope;

public class SetTelescopePosition extends CommandBase {
  private Telescope telescope = Telescope.getInstance();

  private double position;

  /** Creates a new SetTelescopePosition. */
  public SetTelescopePosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(telescope);
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescope.setPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescope.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(telescope.getPosition() - position) < Constants.arm.telescope.ALLOWED_ERROR;
  }
}
