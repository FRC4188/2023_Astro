package frc.robot.utils;

public class SuperstructureStates {
    public enum ClawArmStateType {
        STANDBY,
        INTAKING,
        SCORING
    }

    public enum ArmSide {
        ENERGY_CHAIN_TOP,
        ENERGY_CHAIN_BOTTOM
    }

    public record ArmPositions(
            double shoulderNormalRotations,
            double telescopeNormalRotations,
            double wristNormalRotations,
            double shoulderFlippedRotations,
            double telescopeFlippedRotations,
            double wristFlippedRotations
    ) {
    }

    public enum ArmState {
        //Elevator High and Horizontal extended
        EXTENDED_HIGH_CONE(
                new ArmPositions(
                        44.0, 1.22, 87.0,
                        -40.0, 1.22, -92.0
                ),
                ClawArmStateType.SCORING
        ),
        EXTENDED_HIGH_CUBE(
                new ArmPositions(
                        55.0, 1.25, 120,
                        -51.0, 1.15, -20.0
                ),
                ClawArmStateType.SCORING
        ),
        //Elevator Mid and Horizontal extended
        EXTENDED_MID_CONE(
                new ArmPositions(
                        42.0, 0.76, 100.0,
                        -40.0, 0.76, -100.0
                ),
                ClawArmStateType.SCORING
        ),
        EXTENDED_MID_CUBE(
                new ArmPositions(
                        55.0, 0.5, 120,
                        -51.0, 0.5, -20.0
                ),
                ClawArmStateType.SCORING
        ),
        EXTENDED_LOW_CONE(
                new ArmPositions(
                        60.0, 0.1875, 100.0,
                        -58.0, 0.1875, -100.0
                ),
                ClawArmStateType.SCORING
        ),
        EXTENDED_LOW_CUBE(
                new ArmPositions(
                        75.0, 0.1875, 100.0,
                        -73.0, 0.1875, -20.0
                ),
                ClawArmStateType.SCORING
        ),
        SINGLE_SUB_CONE(
                new ArmPositions(
                        90.0, 0.1875, -30.0,
                        -88.0, 0.1875, 30.0
                ),
                ClawArmStateType.INTAKING
        ),
        SINGLE_SUB_CUBE(
                new ArmPositions(
                        49.0, 0.1875, 100.0,
                        -84.0, 0.1875, 112.0
                ),
                ClawArmStateType.INTAKING
        ),
        //Elevator Platform and Horizontal extended
        DOUBLE_SUB_CONE(
                new ArmPositions(
                        14.0, 0.1875, 82.0,
                        -12.0, 0.1875, -82.0
                ),
                ClawArmStateType.INTAKING
        ),
        DOUBLE_SUB_CUBE(
                new ArmPositions(
                        45.0, 0.8, 126.0,
                        -40.0, 0.1875, 10.0
                ),
                ClawArmStateType.INTAKING
        ),
        //Elevator at normal height
        STANDBY(
                new ArmPositions(
                        0, 0.1875, 110,
                        0, 0.1875, 110
                ),
                ClawArmStateType.STANDBY
        ),
        CONE(
                new ArmPositions(
                        81.0, 0.1875, 57.0,
                        -76.5, 0.1875, -57.0
                ),
                ClawArmStateType.INTAKING
        ),
        CUBE(
                new ArmPositions(
                        102.0, 0.45, 125.0,
                        -113.0, 0.25, 35.0
                ),
                ClawArmStateType.INTAKING
        ),
        TIPPED_CONE(
                new ArmPositions(
                        116, 0.1875, 7.0,
                        -117, 0.1875, 5.0
                ),
                ClawArmStateType.INTAKING
        ),
        BACK_TIPPED_CONE(
                new ArmPositions(
                        86.0, 0.5, 125.0,
                        -85.0, 0.5, -125.0
                ),
                ClawArmStateType.INTAKING
        ),
        YOSHI_CUBE(
                new ArmPositions(
                        90, 0.3, -10,
                        90, 0.5, 90
                ),
                ClawArmStateType.INTAKING
        );

        final ArmPositions armPositions;
        final ClawArmStateType clawArmStateType;

        public ArmPositions getArmPositions() {
            return armPositions;
        }

        public ClawArmStateType getClawArmStateType() {
            return clawArmStateType;
        }

        ArmState(
                final ArmPositions armPositions,
                final ClawArmStateType clawArmStateType
        ) {
            this.armPositions = armPositions;
            this.clawArmStateType = clawArmStateType;
        }
    }

    public enum ClawGamePiece {
        CONE(1),
        CUBE(-1);

        final double scalar;

        ClawGamePiece(final double scalar) {
            this.scalar = scalar;
        }

        public double getScalar() {
            return scalar;
        }
    }

    public enum ClawState {
        STANDBY(0, ClawArmStateType.STANDBY),
        INTAKE(1.0, ClawArmStateType.INTAKING),
        OUTTAKE(-1.0, ClawArmStateType.SCORING);

        final double rollerDutyCycle;
        final ClawArmStateType clawArmStateType;

        public double rollerDutyCycle() {
            return rollerDutyCycle;
        }

        public ClawArmStateType getClawArmStateType() {
            return clawArmStateType;
        }

        ClawState(final double rollerDutyCycle, final ClawArmStateType clawArmStateType) {
            this.rollerDutyCycle = rollerDutyCycle;
            this.clawArmStateType = clawArmStateType;
        }
    }
}
