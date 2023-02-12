package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import csplib.inputs.CSP_Controller;
import csplib.inputs.CSP_Controller.Scale;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.FollowPathPlanner;
import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.subsystems.sensors.Sensors;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Swerve drivetrain = Swerve.getInstance();
  private Sensors sensors = Sensors.getInstance();

  private CSP_Controller pilot = new CSP_Controller(0);

  private SendableChooser<SequentialCommandGroup> autoChooser =
      new SendableChooser<SequentialCommandGroup>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set the default commands
    setDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();

    smartdashboardButtons();
    // Add options to the auto chooser
    addChooser();
  }

  private void setDefaultCommands() {

    drivetrain.setDefaultCommand(
        new RunCommand(
            () ->
                drivetrain.drive(
                    pilot.getLeftY(Scale.SQUARED),
                    pilot.getLeftX(Scale.SQUARED),
                    pilot.getRightX(Scale.CUBED)),
            drivetrain));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    pilot.getAButtonObj().whileTrue(new InstantCommand(() -> sensors.setPigeonAngle(new Rotation3d()), sensors));
  }

  private void smartdashboardButtons() {
    // SmartDashboard.putData(
    //     "Set Velocity",
    //     new RunCommand(
    //         () -> drivetrain.setVelocity(SmartDashboard.getNumber("Set Drive Velocity", 0)),
    //         drivetrain));
    // SmartDashboard.putData(
    //     "Set Angle",
    //     new RunCommand(
    //         () -> drivetrain.setAngle(SmartDashboard.getNumber("Set Drive Angle", 0)), drivetrain));
    SmartDashboard.putData(
        "Set Zero", new InstantCommand(() -> drivetrain.zeroPower(), drivetrain));
  }
  ;

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Pose1ToElem2", new PathConstraints(1, 0.5));
    Pose3d start = new Pose3d(examplePath.getInitialPose().getX(), examplePath.getInitialHolonomicPose().getY(), 0.0, new Rotation3d());
    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.setPose(start)),
      new FollowPathPlanner(examplePath));
  }
}
