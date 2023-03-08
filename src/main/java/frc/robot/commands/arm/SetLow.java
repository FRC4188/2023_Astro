// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class SetLow extends CommandBase {
  private Arm arm = Arm.getInstance();

  private BooleanSupplier isCube;
  private double[] config = Constants.arm.configs.LOW;

  /** Creates a new SetHigh. */
  public SetLow(BooleanSupplier isCube) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm.getShoulder(), arm.getTelescope(), arm.getWrist());
    this.isCube = isCube;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setToScore(config[0], config[1], isCube.getAsBoolean());
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
