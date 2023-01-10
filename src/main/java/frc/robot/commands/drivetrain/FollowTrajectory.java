package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowTrajectory extends CommandBase {

  Trajectory trajectory;
  Rotation2d rotation;

  Drivetrain drivetrain = Drivetrain.getInstance();
  //Odometry odometry = Odometry.getInstance();

  private static HolonomicDriveController controller;
  private static Timer timer = new Timer();

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Trajectory trajectory, Rotation2d rotation) {
    addRequirements(drivetrain);

    this.trajectory = trajectory;
    this.rotation = rotation;

    ProfiledPIDController thetaController = Constants.drivetrain.thetaPID.thetaPID;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    controller = new HolonomicDriveController(Constants.drivetrain.xPID.xPID, Constants.drivetrain.yPID.yPID, thetaController);
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
    drivetrain.setChassisSpeeds(controller.calculate(drivetrain.getPose(), trajectory.sample(timer.get()), rotation));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}