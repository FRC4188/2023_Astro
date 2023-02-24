package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class Wrist {

    private CSP_SparkMax motor = new CSP_SparkMax(Constants.ids.WRIST);

    private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.WRIST_ENCODER);

    private ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, null);

    private ArmFeedforward ff = new ArmFeedforward(Constants.arm.Wrist.kS, Constants.arm.Wrist.kG, Constants.arm.Wrist.kV);
    
    public Wrist(){
        init();
    }

    private void init() {
        encoder.configFactoryDefault();
        encoder.clearStickyFaults();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(-Constants.arm.Wrist.ZERO);

        motor.setBrake(true);
        motor.setPosition(encoder.getAbsolutePosition());
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.Wrist.UPPER_LIMIT);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.Wrist.LOWER_LIMIT);
        motor.setPIDF(Constants.arm.Wrist.kP, Constants.arm.Wrist.kI, Constants.arm.Wrist.kD, Constants.arm.Wrist.kF);

        motor.setMotionPlaning(Constants.arm.Wrist.minVel, Constants.arm.Wrist.maxVel, Constants.arm.Wrist.allowedErr);
    }

    public void setAngle(double angle){
        motor.setPosition(angle);;
    }

    public void setPID(double kP, double kI, double kD){
        pid.setPID(kP, kI, kD);
    } 

    public double getAngle(){
        return encoder.getAbsolutePosition();
    }

   
}
