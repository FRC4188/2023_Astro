package frc.robot.subsystems.arm;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import csplib.motors.CSP_SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Elevator {

    private CSP_SparkMax liftMotor = new CSP_SparkMax(Constants.ids.ELEVATOR_MOTOR);

    private ProfiledPIDController pid = new ProfiledPIDController(Constants.arm.elevator.kP, Constants.arm.elevator.kI, Constants.arm.elevator.kD, Constants.arm.elevator.constriants);

    private ElevatorFeedforward ff = new ElevatorFeedforward(Constants.arm.elevator.kS, Constants.arm.elevator.kG, Constants.arm.elevator.kV);

    private DigitalInput limitSwitch = new DigitalInput(0);

    int counter;    


    public Elevator(){
        init();
    }

    public void init(){
        liftMotor.setScalar(1 / Constants.arm.elevator.TICKS_PER_METER);
        liftMotor.setBrake(true);
        liftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        liftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        liftMotor.setPIDF(Constants.arm.elevator.kP, Constants.arm.elevator.kI, Constants.arm.elevator.kD, Constants.arm.elevator.kF);
    }

    public void resetAngle(){
        if(!limitSwitch.get()){
            liftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
            liftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
            liftMotor.set(-0.2);
        }
        else{
            liftMotor.setEncoder(0.0);
            liftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
            liftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            liftMotor.set(0.0);
        }
    }

    public void setPID(double kP, double kI, double kD){
        pid.setPID(kP, kI, kD);
    }
    

    public void setMotorSpeed(double speed) {
        if (speed > 0) {
            if (limitSwitch.get()) {
                liftMotor.setVoltage(0);
            } else {
                liftMotor.setVoltage(speed);
            }
        }
    }       

    public void setAngle(double goal){
        if (pid.calculate(liftMotor.getPosition(), goal) + ff.calculate(pid.getSetpoint().velocity) > 0) {
            if (limitSwitch.get()) {
                liftMotor.setVoltage(0);
            } else {
                liftMotor.setVoltage(pid.calculate(liftMotor.getPosition(), goal)+ff.calculate(pid.getSetpoint().velocity));
            }
        }
        
    }

    public double getAngle(){
        return(liftMotor.getPosition());
    }
}
