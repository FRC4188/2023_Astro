// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import com.ctre.phoenix.sensors.Pigeon2;

import csplib.utils.Conversions;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Pigeon extends Pigeon2 {
    public Pigeon(int id, String can) {
        super(id, can);
        init();
    }

    public Pigeon(int id) {
        super(id);
        init();
    }

    private void init() {
        super.configFactoryDefault();
        super.clearStickyFaults();
        reset();
    }

    public void reset() {
        super.setYaw(0.0);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(super.getYaw());
    }

}
