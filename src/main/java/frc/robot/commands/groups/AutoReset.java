//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.commands.groups;
//
//import edu.wpi.first.wpilibj2.command.*;
//import frc.robot.Constants;
//import frc.robot.commands.arm.shoulder.HoldShoulder;
//import frc.robot.commands.arm.shoulder.SetShoulderAngle;
//import frc.robot.commands.arm.telescope.SetTelescopePosition;
//import frc.robot.commands.arm.telescope.ZeroTelescope;
//import frc.robot.commands.arm.wrist.SetWristAngle;
//import frc.robot.subsystems.arm.Shoulder;
//import frc.robot.subsystems.claw.ClawOLD;
//
//// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
//// information, see:
//// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//public class AutoReset extends ParallelCommandGroup {
//    private final ClawOLD clawOLD = ClawOLD.getInstance();
//    private final Shoulder shoulder = Shoulder.getInstance();
//    private final double shoulderAngle = Constants.arm.configs.RESET[0];
//    private final double telescopeLength = Constants.arm.configs.RESET[1];
//    private final double wristAngle = Constants.arm.configs.RESET[2];
//
//    /**
//     * Creates a new Reset.
//     */
//    public AutoReset() {
//        // Add your commands in the addCommands() call, e.g.
//        // addCommands(new FooCommand(), new BarCommand());
//        addCommands(
//                new SequentialCommandGroup(
//                        new ParallelDeadlineGroup(new ZeroTelescope(), new HoldShoulder()),
//                        new SetShoulderAngle(shoulderAngle)
//                                .until(() -> shoulder.atGoal(shoulderAngle))
//                                .andThen(new InstantCommand(() -> shoulder.disable())),
//                        new ParallelCommandGroup(
//                                new SetShoulderAngle(shoulderAngle)
//                                        .until(() -> shoulder.atGoal(shoulderAngle))
//                                        .andThen(new InstantCommand(() -> shoulder.disable())),
//                                new SetTelescopePosition(telescopeLength))),
//                new ConditionalCommand(
//                        new SetWristAngle(-wristAngle), new SetWristAngle(wristAngle), shoulder::getIsFlipped),
//                new InstantCommand(() -> clawOLD.disable(), clawOLD));
//    }
//}
