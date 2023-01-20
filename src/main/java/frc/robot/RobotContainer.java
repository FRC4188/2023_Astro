package frc.robot;

import csplib.inputs.CSPController;
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
  
  private CSPController pilot = new CSPController(0);

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
 

  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {

  }

  private void smartdashboardButtons() {
    SmartDashboard.putData("Set Velocity", new InstantCommand(() -> drivetrain.setVelocity(SmartDashboard.getNumber("Set Drive Velocity", 0)), drivetrain));
    SmartDashboard.putData("Set Angle", new InstantCommand(() -> drivetrain.setAngle(SmartDashboard.getNumber("Set Drive Angle", 0)), drivetrain));

    SmartDashboard.putData("Set Speed PIDs", new InstantCommand(() -> drivetrain.setSpeedPIDs(
      SmartDashboard.getNumber("Speed kP", 0), 
      SmartDashboard.getNumber("Speed kI", 0), 
      SmartDashboard.getNumber("Speed kD", 0), 
      SmartDashboard.getNumber("Speed kF", 0)), drivetrain));

      SmartDashboard.putData("Set Angle PIDs", new InstantCommand(() -> drivetrain.setSpeedPIDs(
        SmartDashboard.getNumber("Angle kP", 0), 
        SmartDashboard.getNumber("Angle kI", 0), 
        SmartDashboard.getNumber("Angle kD", 0), 
        SmartDashboard.getNumber("Angle kF", 0)), drivetrain));
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
