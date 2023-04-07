package frc.robot;

import csplib.inputs.CSP_Controller;
import csplib.inputs.CSP_Controller.Scale;
import csplib.utils.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoConfigs;
import frc.robot.commands.arm.SetFlip;
import frc.robot.commands.arm.SetFloor;
import frc.robot.commands.arm.SetPosition;
import frc.robot.commands.groups.Reset;
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
  private Claw claw = Claw.getInstance();
  private Shoulder shoulder = Shoulder.getInstance();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private double currentAngle = 0.0;

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
                    pilot.rightBumper().getAsBoolean()
                        ? pilot.getLeftY(Scale.LINEAR) * 0.5
                        : pilot.getLeftY(Scale.LINEAR),
                    pilot.rightBumper().getAsBoolean()
                        ? pilot.getLeftX(Scale.LINEAR) * 0.5
                        : pilot.getLeftX(Scale.LINEAR),
                    pilot.rightBumper().getAsBoolean()
                        ? pilot.getRightX(Scale.SQUARED) * 0.1
                        : pilot.getRightX(Scale.SQUARED)),
            drivetrain));
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    pilot
        .getAButton()
        .onTrue(
            new InstantCommand(
                () -> Sensors.getInstance().setPigeonAngle(new Rotation2d()),
                Sensors.getInstance()));
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

    // copilot
    //     .getRightTButton().onTrue(
    //         new InstantCommand(() -> {currentAngle = shoulder.getAngle();})
    //         .andThen(
    //             new SetPosition(currentAngle + (shoulder.getIsFlipped() ? -1 : 1), Telescope.getInstance().getPosition(), Wrist.getInstance().getMotorAngle()))
    //         );

    // copilot
    //     .getLeftBumperButton().onTrue(
    //         new InstantCommand(() -> {currentAngle = shoulder.getAngle();})
    //         .andThen(
    //             new SetPosition(currentAngle - (shoulder.getIsFlipped() ? -1 : 1), Telescope.getInstance().getPosition(), Wrist.getInstance().getMotorAngle()))
    //         );
    // copilot
    //     .getRightTButton()
    //     .onTrue(new SetFloor(Constants.arm.configs.BACK_TIPPED_CONE,
    // Constants.arm.configs.BACK_TIPPED_CONE));

    // copilot.getRightBumperButton().debounce(0.05).onTrue(new SetCube());

    // copilot.getLeftBumperButton().debounce(0.05).onTrue(new SetFlip().andThen(new Reset()));

    // copilot.getBackButton().onTrue(new Reset());
    // copilot.getStartButton().onTrue(new Reset());

    // Raymond here we'll see if i like this

    copilot
        .getLeftBumperButton()
        .debounce(0.05)
        .onTrue(new InstantCommand(() -> claw.setIsCube(false)));
    copilot
        .getRightBumperButton()
        .debounce(0.05)
        .onTrue(new InstantCommand(() -> claw.setIsCube(true)));

    copilot.getBackButton().onTrue(new SetFlip().andThen(new Reset()));
    copilot.getStartButton().onTrue(new Reset());
  }

  private void smartdashboardButtons() {}

  private void addChooser() {
    autoChooser.setDefaultOption("Do Nothng", new SequentialCommandGroup());
    autoChooser.addOption(
        "High Perfect Auto",
        AutoBuilder.buildAuto(
            "High Perfect Auto", AutoConfigs.EVENTS, AutoConfigs.PerfectAuto.CONSTRAINTS));

    autoChooser.addOption(
        "Balance",
        AutoBuilder.buildAuto("Balance", AutoConfigs.EVENTS, AutoConfigs.PerfectAuto.CONSTRAINTS));
    autoChooser.addOption(
        "Red Bump 2.5",
        AutoBuilder.buildAuto("RBump2.5", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Red Bump 2.5P",
    //     AutoBuilder.buildAuto("RBump2.5P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Red Bump 2",
    //     AutoBuilder.buildAuto("RBump2", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    autoChooser.addOption(
        "Red Bump 2P",
        AutoBuilder.buildAuto("RBump2P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Red Flat 2.5P",
    //     AutoBuilder.buildAuto("RFlat2.5P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Red Flat 2",
    //     AutoBuilder.buildAuto("RFlat2", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    autoChooser.addOption(
        "Red Flat 2P",
        AutoBuilder.buildAuto("RFlat2P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    autoChooser.addOption(
        "Red Flat 3",
        AutoBuilder.buildAuto("RFlat3", AutoConfigs.EVENTS, AutoConfigs.RFlat3.CONSTRAINTS));
    autoChooser.addOption(
        "Red Mid 1.5P",
        AutoBuilder.buildAuto("RMid1.5P", AutoConfigs.EVENTS, AutoConfigs.RMid15P.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Red Mid 2P",
    //     AutoBuilder.buildAuto("RMid2P", AutoConfigs.EVENTS, AutoConfigs.RMid2P.CONSTRAINTS));

    autoChooser.addOption(
        "Blue Bump 2.5",
        AutoBuilder.buildAuto("BBump2.5", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Blue Bump 2.5P",
    //     AutoBuilder.buildAuto("BBump2.5P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Blue Bump 2",
    //     AutoBuilder.buildAuto("BBump2", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    autoChooser.addOption(
        "Blue Bump 2P",
        AutoBuilder.buildAuto("BBump2P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Blue Flat 2.5P",
    //     AutoBuilder.buildAuto("BFlat2.5P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Blue Flat 2",
    //     AutoBuilder.buildAuto("BFlat2", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    autoChooser.addOption(
        "Blue Flat 2P",
        AutoBuilder.buildAuto("BFlat2P", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
    autoChooser.addOption(
        "Blue Flat 3",
        AutoBuilder.buildAuto("BFlat3", AutoConfigs.EVENTS, AutoConfigs.RFlat3.CONSTRAINTS));
    autoChooser.addOption(
        "Blue Mid 1.5P",
        AutoBuilder.buildAuto("BMid1.5P", AutoConfigs.EVENTS, AutoConfigs.RMid15P.CONSTRAINTS));
    // autoChooser.addOption(
    //     "Blue Mid 2P",
    //     AutoBuilder.buildAuto("BMid2P", AutoConfigs.EVENTS, AutoConfigs.RMid2P.CONSTRAINTS));

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
