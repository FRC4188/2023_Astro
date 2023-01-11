package csplib.inputs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class CSPController extends XboxController {
    public enum Scale {
        LINEAR, SQUARED, CUBED
    }

    private static final double DEADBAND = 0.15;
    private static final double TRIGGER_THRESHOLD = 0.6;

    public CSPController(int port) {
        super(port);
    }

    private double getOutput(double input, Scale scale) {
        if (Math.abs(input) > DEADBAND) {
            if (scale == Scale.SQUARED) return Math.signum(input) * Math.pow(input, 2);
            else if (scale == Scale.CUBED) return Math.pow(input, 3);
            else return input;
        } else {
            return 0;
        }
    }

    public double getRightY(Scale scale) {
        return -getOutput(getRightY(), scale);
    }

    public double getRightX(Scale scale) {
        return getOutput(getRightX(), scale);
    }

    public double getLeftY(Scale scale) {
        return -getOutput(getLeftY(), scale);
    }

    public double getLeftX(Scale scale) {
        return -getOutput(getLeftX(), scale);
    }

    public JoystickButton getAButtonObj() {
        return new JoystickButton(this, Button.kA.value);
    }

    public JoystickButton getBButtonObj() {
        return new JoystickButton(this, Button.kB.value);
    }

    public JoystickButton getXButtonObj() {
        return new JoystickButton(this, Button.kX.value);
    }

    public JoystickButton getYButtonObj() {
        return new JoystickButton(this, Button.kY.value);
    }

    public JoystickButton getStartButtonObj() {
        return new JoystickButton(this, Button.kStart.value);
    }

    public JoystickButton getBackButtonObj() {
        return new JoystickButton(this, Button.kBack.value);
    }

    public JoystickButton getLBButtonObj() {
        return new JoystickButton(this, Button.kLeftBumper.value);
    }

    public JoystickButton getRBButtonObj() {
        return new JoystickButton(this, Button.kLeftBumper.value);
    }

    public JoystickButton getLSButtonObj() {
        return new JoystickButton(this, Button.kLeftStick.value);
    }

    public JoystickButton getRSButtonObj() {
        return new JoystickButton(this, Button.kRightStick.value);
    }

    public POVButton getDpadUpButtonObj() {
        return new POVButton(this, 0);
    }

    public POVButton getDpadDownButtonObj() {
        return new POVButton(this, 180);
    }

    public POVButton getDpadRightButtonObj() {
        return new POVButton(this, 90);
    }

    public POVButton getDpadLeftButtonObj() {
        return new POVButton(this, 270);
    }

    public double getRightT(Scale scale) {
        return getOutput(getRightTriggerAxis() > TRIGGER_THRESHOLD ? getRightTriggerAxis() : 0.0, scale);
    }

    public double getLeftT(Scale scale) {
        return getOutput(getLeftTriggerAxis() > TRIGGER_THRESHOLD ? getLeftTriggerAxis() : 0.0, scale);
    }

    public void setRumble(double intensity) {
        if (DriverStation.isDisabled()) {
            setRumble(RumbleType.kRightRumble, 0);
            setRumble(RumbleType.kLeftRumble, 0);
        } else {
            setRumble(RumbleType.kLeftRumble, intensity);
            setRumble(RumbleType.kRightRumble, intensity);
        }
    }
}
