package frc.robot.AidenLib.control;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.AidenLib.math.Data;
import frc.robot.AidenLib.math.WeightedFusion;

/** A class which works similarly to {@link Odometry3D} but in addition to drivetrain data it also fuses other measures of the robot position. */
public class PoseEstimator3D {
    private DoubleSupplier driveStDev;
    private Odometry3D odometry;

    /**
     * Constructs a {@link PoseEstimator3D} object.
     * @param initialPose The initial position of the robot in <b>m</b>.
     * @param driveStDev The standard deviation of the drive data in <b>m</b>.
     */
    public PoseEstimator3D(Pose3d initialPose, DoubleSupplier driveStDev) {
        this.driveStDev = driveStDev;
        odometry = new Odometry3D(initialPose);
    }

    /**
     * Constructs a {@link PoseEstimator3D} object with <b>new Pose3d()</b> as a default initial position.
     * @param driveStDev The standard deviation of the drive data error in <b>m per robot loop</b>.
     */
    public PoseEstimator3D(DoubleSupplier driveStDev) {
        this(new Pose3d(), driveStDev);
    }

        /**
     * A method to calculate the optimal estimate of the robot's position based on drive odometry as well as other measurements.
     * <p>
     * The {@link Data} arrays may be different lengths.
     * </p>
     * @param speeds Robot relative speeds of the robot in <b>m/s</b>.
     * @param rotEstimate An estimate of the robot heading which may be directly supplied by an IMU but should be the result of the {@link RotationEstimator3D} class.
     * @param xMeasures An array of any length (including 0) representing measurements of the X position by {@link Data} objects in <b>m</b>.
     * @param yMeasures An array of any length (including 0) representing measurements of the Y position by {@link Data} objects in <b>m</b>.
     * @param zMeasures An array of any length (including 0) representing measurements of the Z position by {@link Data} objects in <b>m</b>.
     * @return The optimal estimate of the robot position based on the given measurements in <b>m</b>.
     */
    public Pose3d estimate(Translation3d speeds, Rotation3d rotEstimate, LinkedList<Data> xMeasures, LinkedList<Data> yMeasures, LinkedList<Data> zMeasures) {
        ArrayList<Data> pVals = new ArrayList<Data>(List.of());

        Pose3d pose = odometry.update(speeds, rotEstimate);

        double stDev = driveStDev.getAsDouble();

        xMeasures.add(new Data(pose.getX(), stDev));
        yMeasures.add(new Data(pose.getY(), stDev));
        zMeasures.add(new Data(pose.getZ(), stDev));

        pVals = WeightedFusion.calculateParameterized(List.of(xMeasures, yMeasures, zMeasures));
        pose = new Pose3d(pVals.get(0).value, pVals.get(1).value, pVals.get(2).value, rotEstimate);
        odometry.setPose(pose);

        return odometry.getPose();
    }

    /**
     * A method to calculate the optimal estimate of the robot's position based on drive odometry as well as other measurements.
     * <p>
     * The {@link Data} arrays may be different lengths.
     * </p>
     * @param speeds Robot relative speeds of the robot in <b>m/s</b>.
     * @param rotEstimate An estimate of the robot heading which may be directly supplied by an IMU but should be the result of the {@link RotationEstimator3D} class.
     * @param xMeasures An array of any length (including 0) representing measurements of the X position by {@link Data} objects in <b>m</b>.
     * @param yMeasures An array of any length (including 0) representing measurements of the Y position by {@link Data} objects in <b>m</b>.
     * @param zMeasures An array of any length (including 0) representing measurements of the Z position by {@link Data} objects in <b>m</b>.
     * @return The optimal estimate of the robot position based on the given measurements in <b>m</b>.
     */
    public Pose3d estimate(ChassisSpeeds speeds, Rotation3d rotEstimate, LinkedList<Data> xMeasures, LinkedList<Data> yMeasures, LinkedList<Data> zMeasures) {
        return estimate(new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0), rotEstimate, xMeasures, yMeasures, zMeasures);
    }

    public void setPose(Pose3d pose) {
        odometry.setPose(pose);
    }

    public Pose3d getPose() {
        return odometry.getPose();
    }
}
