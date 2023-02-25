package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;

public class Wrist {

    private CSP_SparkMax motor = new CSP_SparkMax(Constants.ids.WRIST);
    private WPI_CANCoder encoder = new WPI_CANCoder(Constants.ids.WRIST_ENCODER);

    public Wrist(){
        init();
    }

    private void init() {
        encoder.configFactoryDefault();
        encoder.clearStickyFaults();
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        encoder.setPosition(0.0);
        encoder.configSensorDirection(false);
        encoder.configMagnetOffset(-Constants.arm.wrist.ZERO);

        motor.setScalar(1 / Constants.arm.wrist.TICKS_PER_DEGREE);
        motor.setBrake(true);
        motor.setPosition(encoder.getAbsolutePosition());
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.arm.wrist.UPPER_LIMIT);
        motor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.arm.wrist.LOWER_LIMIT);

        motor.setMotionPlaning(Constants.arm.telescope.MINVEL, Constants.arm.telescope.MAXVEL);
        motor.setError(Constants.arm.wrist.ALLOWERROR);
    }

    public void setAngle(double goal){
        motor.setPosition(goal);
    }

    public void setPID(double kP, double kI, double kD, double kF){
        motor.setPIDF(kP, kI, kD, kF);
    }

    public double getAngle(){
        return encoder.getAbsolutePosition();
    }

   
}
