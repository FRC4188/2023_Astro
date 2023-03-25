package frc.robot;

import csplib.inputs.CSP_Controller;
import csplib.inputs.CSP_Controller.Scale;
import csplib.utils.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.arm.SetCube;
import frc.robot.commands.arm.SetFlip;
import frc.robot.commands.arm.SetFloor;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.groups.Reset;
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
  private Claw claw = Claw.getInstance();

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
                    pilot.getLeftY(Scale.QUARTIC),
                    pilot.getLeftX(Scale.QUARTIC),
                    pilot.getRightX(Scale.SQUARED)),
            drivetrain));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    pilot
        .getAButton()
        .onTrue(
            new InstantCommand(() -> Sensors.getInstance().resetPigeon(), Sensors.getInstance()));

    pilot
        .getLeftTButton()
        .whileTrue(new RunCommand(() -> claw.outtake(), claw))
        .onFalse(new InstantCommand(() -> claw.disable(), claw));
    pilot
        .getRightTButton()
        .whileTrue(new RunCommand(() -> claw.intake(), claw))
        .onFalse(new InstantCommand(() -> claw.disable(), claw));
    
    copilot
        .getAButton()
        .onTrue(new SetFloor(Constants.arm.configs.FLOOR_CUBE, Constants.arm.configs.FLOOR_CONE));

    copilot
        .getXButton()
        .onTrue(new SetPosition(Constants.arm.configs.SS_CUBE, Constants.arm.configs.SS_CONE));

    copilot
        .getYButton()
        .onTrue(new SetPosition(Constants.arm.configs.DS_CUBE, Constants.arm.configs.DS_CONE));
    
    copilot
        .getBButton()
        .onTrue(new SetFloor(Constants.arm.configs.FLOOR_CUBE, Constants.arm.configs.TIPPED_CONE));

    copilot
        .getUpButton()
        .onTrue(new SetPosition(Constants.arm.configs.HIGH_CUBE, Constants.arm.configs.HIGH_CONE));

    copilot
        .getRightButton()
        .onTrue(new SetPosition(Constants.arm.configs.MID_CUBE, Constants.arm.configs.MID_CONE));

    copilot
        .getLeftButton()
        .onTrue(new SetPosition(Constants.arm.configs.MID_CUBE, Constants.arm.configs.MID_CONE));

    copilot
        .getDownButton()
        .onTrue(new SetPosition(Constants.arm.configs.LOW_CUBE, Constants.arm.configs.LOW_CONE));

    copilot
        .getRightBumperButton() 
        .debounce(0.15)
        .toggleOnTrue(new SetCube());

    copilot
        .getLeftBumperButton()
        .debounce(0.15)
        .toggleOnTrue(new SetFlip());

    copilot.getBackButton().onTrue(new Reset());
    copilot.getStartButton().onTrue(new Reset());
  }

  private void smartdashboardButtons() {
    // SmartDashboard.putData(
    //     "Set Drive Rot PID",
    //     new InstantCommand(
    //         () ->
    //             drivetrain.setRotPID(
    //                 SmartDashboard.getNumber("Rot P", 0),
    //                 SmartDashboard.getNumber("Rot I", 0),
    //                 SmartDashboard.getNumber("Rot D", 0))));

    // SmartDashboard.putData(
    //     "Set Telescope PID",
    //     new InstantCommand(
    //         () ->
    //             telescope.setPID(
    //                 SmartDashboard.getNumber("Telescope P", 0),
    //                 SmartDashboard.getNumber("Telescope I", 0),
    //                 SmartDashboard.getNumber("Telescope D", 0))));

    // SmartDashboard.putData(
    //     "Set Drive Rot",
    //     new RunCommand(
    //         () -> drivetrain.setRotation(SmartDashboard.getNumber("Drive Rot", 0)), drivetrain));

    // SmartDashboard.putData(
    //     "Set Zero", new InstantCommand(() -> drivetrain.zeroPower(), drivetrain));
  }

  private void addChooser() {
    autoChooser.setDefaultOption(
        "High Perfect Auto",
        AutoBuilder.buildAuto(
            "High Perfect Auto", AutoConfigs.EVENTS, AutoConfigs.PerfectAuto.CONSTRAINTS));
    autoChooser.addOption(
        "3-2P", AutoBuilder.buildAuto("3-2P", AutoConfigs.EVENTS, AutoConfigs.three2P.CONSTRAINTS));
    autoChooser.addOption(
        "3-1P", AutoBuilder.buildAuto("3-1P", AutoConfigs.EVENTS, AutoConfigs.three1P.CONSTRAINTS));
    autoChooser.addOption(
        "High Perfect Auto",
        AutoBuilder.buildAuto(
            "High Perfect Auto", AutoConfigs.EVENTS, AutoConfigs.PerfectAuto.CONSTRAINTS));
    autoChooser.addOption(
        "The Perfect Auto",
        AutoBuilder.buildAuto(
            "Perfect Auto", AutoConfigs.EVENTS, AutoConfigs.PerfectAuto.CONSTRAINTS));
    autoChooser.addOption(
        "3-1", AutoBuilder.buildAuto("3-1", AutoConfigs.EVENTS, AutoConfigs.three1P.CONSTRAINTS));
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
