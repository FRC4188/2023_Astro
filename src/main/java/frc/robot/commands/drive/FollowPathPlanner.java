// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Swerve;

public class FollowPathPlanner extends CommandBase {

  PathPlannerTrajectory trajectory;

  frc.robot.subsystems.drivetrain.Swerve swerve = Swerve.getInstance();

  private static HolonomicDriveController controller;
  private static Timer timer = new Timer();

  /** Creates a new FollowTrajectory. */
  public FollowPathPlanner(PathPlannerTrajectory trajectory) {
    addRequirements(swerve);

    this.trajectory = trajectory;

    ProfiledPIDController thetaController = Constants.drive.thetaPID.thetaPID;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller =
        new HolonomicDriveController(
            Constants.drive.xPID.xPID, Constants.drive.yPID.yPID, thetaController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
    System.out.println(state.toString());
    ChassisSpeeds speeds = controller.calculate(swerve.getPose().toPose2d(), state, state.holonomicRotation);
    speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    swerve.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.zeroPower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}
