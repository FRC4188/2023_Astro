package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import csplib.motors.CSP_Motor;
import csplib.motors.CSP_SparkMax;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
    private static Claw claw;

    public static synchronized Claw getInstance(){
        if (claw == null) claw = new Claw();
        return claw;
    }

    private TalonSRX motor = new TalonSRX(0);
    public Claw() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

      public void set(TalonSRXControlMode mode, double power) {
        motor.set(mode, power);
      }





}
