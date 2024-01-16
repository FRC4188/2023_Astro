package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.ArmClawTeleop;
import frc.robot.commands.drive.SwerveDriveTeleop;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.drivetrain.Swerve;
import frc.robot.wrappers.vision.PhotonVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //Controllers
    public final CommandXboxController pilot;
    public final CommandXboxController copilot;

    //Subsystems
    public final Swerve swerve;
    public final Arm arm;
    public final Claw claw;

    //Vision
    public final PhotonVision photonVision;

    //Commands
    public final SwerveDriveTeleop swerveDriveTeleop;
    public final ArmClawTeleop armClawTeleop;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        pilot = new CommandXboxController(RobotMap.PILOT_PORT);
        copilot = new CommandXboxController(RobotMap.COPILOT_PORT);

        swerve = new Swerve(
                Constants.CURRENT_MODE,
                HardwareConstants.FRONT_LEFT_MODULE,
                HardwareConstants.FRONT_RIGHT_MODULE,
                HardwareConstants.BACK_LEFT_MODULE,
                HardwareConstants.BACK_RIGHT_MODULE
        );

        arm = new Arm(
                Constants.CURRENT_MODE,
                HardwareConstants.ARM
        );

        claw = new Claw(
                Constants.CURRENT_MODE,
                HardwareConstants.CLAW
        );

        photonVision = new PhotonVision(swerve, swerve.getPoseEstimator());

        swerveDriveTeleop = new SwerveDriveTeleop(swerve, pilot.getHID());
        armClawTeleop = new ArmClawTeleop(arm, claw);

        addChooser();
    }


    private void addChooser() {
//        autoChooser.setDefaultOption("Do Nothng", new SequentialCommandGroup());
//
//        autoChooser.addOption(
//                "Blue Flat 3",
//                AutoBuilder.buildAuto("BFlat3", AutoConfigs.EVENTS, AutoConfigs.RFlat3.CONSTRAINTS));
//        autoChooser.addOption(
//                "Blue Mid 1.5P",
//                AutoBuilder.buildAuto("BMid1.5P", AutoConfigs.EVENTS, AutoConfigs.RMid15P.CONSTRAINTS));
//
//        autoChooser.addOption(
//                "Blue Bump 2.5",
//                AutoBuilder.buildAuto("BBump2.5", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
//
//        autoChooser.addOption(
//                "Red Flat 3",
//                AutoBuilder.buildAuto("RFlat3", AutoConfigs.EVENTS, AutoConfigs.RFlat3.CONSTRAINTS));
//
//        autoChooser.addOption(
//                "Red Mid 1.5P",
//                AutoBuilder.buildAuto("RMid1.5P", AutoConfigs.EVENTS, AutoConfigs.RMid15P.CONSTRAINTS));
//
//        autoChooser.addOption(
//                "Red Bump 2.5",
//                AutoBuilder.buildAuto("RBump2.5", AutoConfigs.EVENTS, AutoConfigs.RFlat2.CONSTRAINTS));
//
//        SmartDashboard.putData("Auto Chooser", autoChooser);
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
