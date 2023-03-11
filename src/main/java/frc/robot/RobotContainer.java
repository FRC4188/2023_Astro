package frc.robot;

import csplib.inputs.CSP_Controller;
import csplib.inputs.CSP_Controller.Scale;
import csplib.utils.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoEventMaps;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private CSP_Controller pilot = new CSP_Controller(Constants.controller.PILOT_PORT);
  private CSP_Controller copilot = new CSP_Controller(Constants.controller.COPILOT_PORT);

  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Arm arm = Arm.getInstance();
  private Claw claw = Claw.getInstance();

  private Shoulder shoulder = Shoulder.getInstance();
  private Telescope telescope = Telescope.getInstance();
  private Wrist wrist = Wrist.getInstance();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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
                    pilot.getRightX(Scale.SQUARED)),
            drivetrain));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    claw.setIsCube(copilot.getRightBumperButton().getAsBoolean());

    copilot
        .getRightBumperButton()
        .whileTrue(new InstantCommand(() -> claw.set(0.7), claw))
        .onFalse(new InstantCommand(() -> claw.set(0.0), claw));
    copilot
        .getLeftBumperButton()
        .whileTrue(new InstantCommand(() -> claw.set(-0.7), claw))
        .onFalse(new InstantCommand(() -> claw.set(0.0), claw));

    copilot
        .getYButton()
        .whileTrue(new InstantCommand(() -> shoulder.set(0.3), shoulder))
        .onFalse(new InstantCommand(() -> shoulder.set(0.0), shoulder));
    copilot
        .getAButton()
        .whileTrue(new InstantCommand(() -> shoulder.set(-0.3), shoulder))
        .onFalse(new InstantCommand(() -> shoulder.set(0.0), shoulder));
    copilot
        .getXButton()
        .whileTrue(new InstantCommand(() -> wrist.set(0.3), wrist))
        .onFalse(new InstantCommand(() -> wrist.set(0.0), wrist));
    copilot
        .getBButton()
        .whileTrue(new InstantCommand(() -> wrist.set(-0.3), wrist))
        .onFalse(new InstantCommand(() -> wrist.set(0.0), wrist));
    copilot
        .getUpButton()
        .whileTrue(new InstantCommand(() -> telescope.set(0.3), telescope))
        .onFalse(new InstantCommand(() -> telescope.set(0.0), telescope));
    copilot
        .getDownButton()
        .whileTrue(new InstantCommand(() -> telescope.set(-0.3), telescope))
        .onFalse(new InstantCommand(() -> telescope.set(0.0), telescope));

    copilot.getStartButton().onTrue(new ZeroTelescope());
    copilot
        .getBackButton()
        .onTrue(new RunCommand(() -> telescope.setPosition(0.75), telescope))
        .onFalse(new InstantCommand(() -> telescope.disable(), telescope));
    copilot
        .getRightButton()
        .onTrue(new RunCommand(() -> telescope.setPosition(0.4), telescope))
        .onFalse(new InstantCommand(() -> telescope.disable(), telescope));
  }

  private void smartdashboardButtons() {
    SmartDashboard.putData(
        "Set Drive Rot PID",
        new InstantCommand(
            () ->
                drivetrain.setRotPID(
                    SmartDashboard.getNumber("Rot P", 0),
                    SmartDashboard.getNumber("Rot I", 0),
                    SmartDashboard.getNumber("Rot D", 0))));

    SmartDashboard.putData(
        "Set Telescope PID",
        new InstantCommand(
            () ->
                Telescope.getInstance()
                    .setPID(
                        SmartDashboard.getNumber("Telescope P", 0),
                        SmartDashboard.getNumber("Telescope I", 0),
                        SmartDashboard.getNumber("Telescope D", 0))));

    SmartDashboard.putData(
        "Set Drive Rot",
        new RunCommand(
            () -> drivetrain.setRotation(SmartDashboard.getNumber("Drive Rot", 0)), drivetrain));

    SmartDashboard.putData(
        "Set Zero", new InstantCommand(() -> drivetrain.zeroPower(), drivetrain));
  }

  private void addChooser() {
    autoChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    autoChooser.addOption(
        "Test",
        AutoBuilder.buildAuto(
            "Test Auto Path", AutoEventMaps.Test.EVENTS, AutoEventMaps.Test.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Test", AutoBuilder.buildAuto("New Path", new HashMap<>(), new PathConstraints(5.0, 1)));
    // autoChooser.addOption(
    //     "B21", AutoBuilder.buildAuto("B21", AutoEventMaps.b1Map, new PathConstraints(5.0, 2)));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
