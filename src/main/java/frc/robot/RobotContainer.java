package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.CSPController;
import frc.robot.utils.CSPController.Scaling;
=======
import csplib.inputs.CSP_Controller;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;

>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Drivetrain drivetrain = Drivetrain.getInstance();
  
<<<<<<< HEAD
  private CSPController pilot = new CSPController(0);
=======
  private CSP_Controller pilot = new CSP_Controller(0);
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf

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
<<<<<<< HEAD
  drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(
      pilot.getLeftY(Scaling.CUBED),
      pilot.getLeftX(Scaling.CUBED),
      pilot.getRightX(Scaling.CUBED),
      pilot.getLeftStickButton()),
      drivetrain)
    ); 
=======
 
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf

  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
<<<<<<< HEAD
 
   
  }

  private void smartdashboardButtons() {
  };

    
  


  
=======

  }

  private void smartdashboardButtons() {
    SmartDashboard.putData("Set Velocity", new InstantCommand(() -> drivetrain.setVelocity(SmartDashboard.getNumber("Set Drive Velocity", 0)), drivetrain));
    SmartDashboard.putData("Set Angle", new InstantCommand(() -> drivetrain.setAngle(SmartDashboard.getNumber("Set Drive Angle", 0)), drivetrain));  
    SmartDashboard.putData("Set Zero", new InstantCommand(() -> drivetrain.zeroPower(), drivetrain));
    SmartDashboard.putData("Set Selected Module", new InstantCommand(() -> drivetrain.setModuleNum((int) SmartDashboard.getNumber("Set Module", 0)), drivetrain));



    SmartDashboard.putData("Set Speed PIDs", new InstantCommand(() -> drivetrain.setSpeedPIDs(
      SmartDashboard.getNumber("Speed kP", 0), 
      SmartDashboard.getNumber("Speed kI", 0), 
      SmartDashboard.getNumber("Speed kD", 0), 
      SmartDashboard.getNumber("Speed kF", 0)), drivetrain));

      SmartDashboard.putData("Set Angle PIDs", new InstantCommand(() -> drivetrain.setAnglePIDs(
        SmartDashboard.getNumber("Angle kP", 0), 
        SmartDashboard.getNumber("Angle kI", 0), 
        SmartDashboard.getNumber("Angle kD", 0), 
        SmartDashboard.getNumber("Angle kF", 0)), drivetrain));

    

  };
>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf

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
