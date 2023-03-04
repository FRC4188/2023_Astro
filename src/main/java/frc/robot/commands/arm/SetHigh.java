// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Wrist;

public class SetHigh extends CommandBase {
  private Arm arm = Arm.getInstance();
  private Shoulder shoulder = Shoulder.getInstance();
  private Telescope telescope = Telescope.getInstance();
  private Wrist wrist = Wrist.getInstance();

  /** Creates a new SetHigh. */
  public SetHigh() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoulder, telescope, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPosition(Constants.arm.configs.HIGH);
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
