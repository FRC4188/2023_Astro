package frc.robot;

import csplib.inputs.CSP_Controller;
import csplib.inputs.CSP_Controller.Scale;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Drivetrain drivetrain = Drivetrain.getInstance();
  
  private CSP_Controller pilot = new CSP_Controller(0);

  private SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<SequentialCommandGroup>();

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
 
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(
      pilot.getLeftY(Scale.SQUARED), 
      pilot.getLeftX(Scale.SQUARED), 
      pilot.getRightX(Scale.SQUARED), 
      true), drivetrain));
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {

  }

  private void smartdashboardButtons() {
    SmartDashboard.putData("Set Velocity", new RunCommand(() -> drivetrain.setVelocity(SmartDashboard.getNumber("Set Drive Velocity", 0)), drivetrain));
    SmartDashboard.putData("Set Angle", new RunCommand(() -> drivetrain.setAngle(SmartDashboard.getNumber("Set Drive Angle", 0)), drivetrain));  
    SmartDashboard.putData("Set Zero", new InstantCommand(() -> drivetrain.zeroPower(), drivetrain));
  };

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
    return autoChooser.getSelected();
  }
}
