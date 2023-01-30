// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroModules extends InstantCommand {

  Swerve drive = Swerve.getInstance();

  public ZeroModules() {
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveModuleState[] states = drive.getModuleStates();
    Preferences.setDouble("mod1zero", -states[0].angle.getDegrees());
    Preferences.setDouble("mod2zero", -states[1].angle.getDegrees());
    Preferences.setDouble("mod3zero", -states[2].angle.getDegrees());
    Preferences.setDouble("mod4zero", -states[3].angle.getDegrees());
    drive.instantiateModules();
  }
}
