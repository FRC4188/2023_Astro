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
import frc.robot.Constants.arm.telescope;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.arm.SetFlip;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.arm.telescope.ZeroTelescope;
import frc.robot.commands.arm.wrist.HoldWrist;
import frc.robot.commands.claw.Intake;
import frc.robot.commands.claw.Outtake;
import frc.robot.commands.groups.IntakeFrom;
import frc.robot.commands.groups.Reset;
import frc.robot.commands.groups.ScoreOn;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Shoulder;
import frc.robot.subsystems.arm.Telescope;
import frc.robot.subsystems.arm.Wrist;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.sensors.Sensors;

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

    // shoulder.setDefaultCommand(
    //     new RunCommand(() -> shoulder.setAngle(shoulder.getAngle() + (copilot.getRightY(Scale.LINEAR) * 15)), shoulder));
    // telescope.setDefaultCommand(
    //   new RunCommand(() -> telescope.setPosition(telescope.getPosition() + (copilot.getRightT(Scale.LINEAR) - copilot.getLeftT(Scale.LINEAR))), telescope)
    // );
    // wrist.setDefaultCommand(
    //   new HoldWrist(() -> (copilot.getLeftY(Scale.LINEAR) * 10))
    // );
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    pilot
        .getAButton()
        .onTrue(
            new InstantCommand(() -> Sensors.getInstance().resetPigeon(), Sensors.getInstance()));

    pilot.getLeftTButton().whileTrue(new RunCommand(() -> claw.outtake(), claw)).onFalse(new InstantCommand(() -> claw.disable(), claw));
    pilot.getRightTButton().whileTrue(new RunCommand(() -> claw.intake(), claw)).onFalse(new InstantCommand(() -> claw.disable(), claw));

    copilot.getAButton().onTrue(new IntakeFrom(Constants.arm.configs.FLOOR_CONE, Constants.arm.configs.FLOOR_CUBE));
    copilot.getXButton().onTrue(new IntakeFrom(Constants.arm.configs.SS_CONE, Constants.arm.configs.SS_CUBE));
    copilot.getYButton().onTrue(new IntakeFrom(Constants.arm.configs.DS_CONE, Constants.arm.configs.DS_CUBE));
    copilot.getBButton().onTrue(new IntakeFrom(Constants.arm.configs.TIPPED_CONE, Constants.arm.configs.FLOOR_CUBE));

    copilot.getUpButton().onTrue(new SetPosition(Constants.arm.configs.HIGH));
    copilot.getRightButton().onTrue(new SetPosition(Constants.arm.configs.MID));
    copilot.getDownButton().onTrue(new SetPosition(Constants.arm.configs.LOW));

    copilot.getLeftBumperButton().debounce(0.3).onTrue(new InstantCommand(() -> claw.setIsCube(false)));
    copilot.getRightBumperButton().debounce(0.3).onTrue(new InstantCommand(() -> claw.setIsCube(true)));

    copilot.getBackButton().onTrue(new Reset());
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
                telescope
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
            "Test Auto Path", AutoConfigs.EVENTS, AutoConfigs.Test.CONSTRAINTS));


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
