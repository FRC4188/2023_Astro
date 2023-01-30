// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drive.ZeroModules;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.utils.CSPController;
import frc.robot.utils.CSPController.Scaling;


public class RobotContainer {

  Swerve drive = Swerve.getInstance();

  private final CSPController pilot =
      new CSPController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive.setDefaultCommand(new RunCommand(() -> drive.drive(pilot.getLeftY(Scaling.SQUARED), pilot.getLeftX(Scaling.SQUARED), pilot.getRightX(Scaling.LINEAR), pilot.getRightY(Scaling.LINEAR)), drive));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    SmartDashboard.putData("Zero Modules", new ZeroModules());
  }

  
  public Command getAutonomousCommand() {
    return null;
  }
}
