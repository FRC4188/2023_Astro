package frc.robot.AidenLib.control;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.AidenLib.Timer;
import frc.robot.AidenLib.math.Derivative;
import frc.robot.AidenLib.math.Integral;
import frc.robot.AidenLib.math.WeightedFusion;
import frc.robot.AidenLib.math.Data;

/** A class which estimates the robot rotation in 3D space based on imu data fused with other measurements. */
public class RotationEstimator3D {

    private Rotation3d lastEstimate;

    private Derivative xRate, yRate, zRate;
    private Timer timer;

    private double imuStdDev;

    /**
     * Constructs a {@link RotationEstimator3D} object.
     * @param imuStDev The standard deviation of the IMU error in <b>radians per robot loop</b>.
     * @param initialRot The intial rotation of the robot.
     */
    public RotationEstimator3D(double imuStDev, Rotation3d initialRot) {
        this.lastEstimate = initialRot;
        this.xRate = new Derivative(initialRot.getX());
        this.yRate = new Derivative(initialRot.getY());
        this.zRate = new Derivative(initialRot.getZ());
        this.timer = new Timer();
        this.imuStdDev = imuStDev;
    }

    /**
     * Constructs a {@link RotationEstimator3D} object using <b>new Pose3d()</b> as the default initial position.
     * @param imuStDev The standard deviation of the IMU error in <b>radians per robot loop</b>.
     */
    public RotationEstimator3D(double imuStDev) {
        this(imuStDev, new Rotation3d());
    }

    /**
     * A method which fuses IMU data with other provided measurements.
     * <p>
     * This method provides an intended input to the <b>estimate()</b> method of the {@link PoseEstimator3D} class.
     * </p>
     * @param imuMeasure The data provided from the IMU in the form of a {@link Rotation3d} object.
     * @param yawMeasures An array of any length (including 0) representing measurements of the yaw rotation by {@link Data} objects in <b>radians</b>.
     * @param pitchMeasures An array of any length (including 0) representing measurements of the pitch rotation by {@link Data} objects in <b>radians</b>.
     * @param rollMeasures An array of any length (including 0) representing measurements of the roll rotation by {@link Data} objects in <b>radians</b>.
     * @return The optimal estimate of the robot rotation.
     */
    public Rotation3d estimate(Rotation3d imuMeasure, LinkedList<Data> xMeasures, LinkedList<Data> yMeasures, LinkedList<Data> zMeasures) {
        List<Data> pVals = List.of(new Data(), new Data(), new Data());

        double dt = timer.getDT();

        imuMeasure = new Rotation3d(
            lastEstimate.getX() + Integral.riemannSum(xRate.getRate(imuMeasure.getX()), dt),
            lastEstimate.getY() + Integral.riemannSum(yRate.getRate(imuMeasure.getY()), dt),
            lastEstimate.getZ() + Integral.riemannSum(zRate.getRate(imuMeasure.getZ()), dt)
        );

        xMeasures.add(new Data(imuMeasure.getX(), imuStdDev));
        yMeasures.add(new Data(imuMeasure.getY(), imuStdDev));
        zMeasures.add(new Data(imuMeasure.getZ(), imuStdDev));

        pVals = WeightedFusion.calculateParameterized(List.of(xMeasures, yMeasures, zMeasures));

        return new Rotation3d(pVals.get(0).value, pVals.get(1).value, pVals.get(2).value);
    }

    public void setRotation(Rotation3d rot) {
        lastEstimate = rot;
    }
}