package csplib.inputs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;

public class CSP_Controller extends XboxController {
    /**
     * Controller input scale
     */
    public enum Scale {
        LINEAR, SQUARED, CUBED
    }

    /**
     * Creates a new CSP_Controller object
     * @param port controller input port 
     */
    public CSP_Controller(int port) {
        super(port);
    }

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> 00e51d0956192522bddea2404088c98a2720d7cf
=======
>>>>>>> 56c362aebdce62bca6010cd79440815143a53295
=======
    /**
     * Calculates joystick output to account for scale and deadband
     * @param input input value
     * @param scale input scale
     * @return adjusted value
     */
>>>>>>> 4cb66e0ea8021806465a430afb2d765de6919b02
    private double getOutput(double input, Scale scale) {
        if (Math.abs(input) > Constants.controller.DEADBAND) {
            if (scale == Scale.SQUARED) return Math.signum(input) * Math.pow(input, 2);
            else if (scale == Scale.CUBED) return Math.pow(input, 3);
            else return input;
        } else {
            return 0;
        }
    }

    /**
     * Get the adjusted value of the right Y joystick
     * @param scale input scale
     * @return adjusted value
     */
    public double getRightY(Scale scale) {
        return -getOutput(getRightY(), scale);
    }

    /**
     * Get the adjusted value of the right x joystick
     * @param scale input scale
     * @return adjusted value
     */
    public double getRightX(Scale scale) {
        return getOutput(getRightX(), scale);
    }

    /**
     * Get the adjusted value of the left Y joystick
     * @param scale input scale
     * @return adjusted value
     */
    public double getLeftY(Scale scale) {
        return -getOutput(getLeftY(), scale);
    }

    /**
     * Get the adjusted value of the left X joystick
     * @param scale input scale
     * @return adjusted value
     */
    public double getLeftX(Scale scale) {
        return -getOutput(getLeftX(), scale);
    }

    /**
     * Get the object of the A button
     * @return A JoystickButton object 
     */
    public JoystickButton getAButtonObj() {
        return new JoystickButton(this, Button.kA.value);
    }

    /**
     * Get the object of the B button
     * @return B JoystickButton object 
     */
    public JoystickButton getBButtonObj() {
        return new JoystickButton(this, Button.kB.value);
    }

    /**
     * Get the object of the X button
     * @return X JoystickButton object 
     */
    public JoystickButton getXButtonObj() {
        return new JoystickButton(this, Button.kX.value);
    }

    /**
     * Get the object of the Y button
     * @return Y JoystickButton object 
     */
    public JoystickButton getYButtonObj() {
        return new JoystickButton(this, Button.kY.value);
    }

    /**
     * Get the object of the Start button
     * @return Start JoystickButton object 
     */
    public JoystickButton getStartButtonObj() {
        return new JoystickButton(this, Button.kStart.value);
    }

    /**
     * Get the object of the Back button
     * @return Back JoystickButton object 
     */
    public JoystickButton getBackButtonObj() {
        return new JoystickButton(this, Button.kBack.value);
    }

    /**
     * Get the object of the LB button
     * @return LB JoystickButton object 
     */
    public JoystickButton getLBButtonObj() {
        return new JoystickButton(this, Button.kLeftBumper.value);
    }

    /**
     * Get the object of the RB button
     * @return RB JoystickButton object 
     */
    public JoystickButton getRBButtonObj() {
        return new JoystickButton(this, Button.kLeftBumper.value);
    }

    /**
     * Get the object of the LS button
     * @return LS JoystickButton object 
     */
    public JoystickButton getLSButtonObj() {
        return new JoystickButton(this, Button.kLeftStick.value);
    }

    /**
     * Get the object of the RS button
     * @return RS JoystickButton object 
     */
    public JoystickButton getRSButtonObj() {
        return new JoystickButton(this, Button.kRightStick.value);
    }

    /**
     * Get the object of the Dpad Up button
     * @return Dpad Up JoystickButton object 
     */
    public POVButton getDpadUpButtonObj() {
        return new POVButton(this, 0);
    }

    /**
     * Get the object of the Dpad Down button
     * @return Dpad Down JoystickButton object 
     */
    public POVButton getDpadDownButtonObj() {
        return new POVButton(this, 180);
    }

    /**
     * Get the object of the Dpad Right button
     * @return Dpad Right JoystickButton object 
     */
    public POVButton getDpadRightButtonObj() {
        return new POVButton(this, 90);
    }

    /**
     * Get the object of the Dpad Left button
     * @return Dpad Left JoystickButton object 
     */
    public POVButton getDpadLeftButtonObj() {
        return new POVButton(this, 270);
    }

    /**
     * Get the adjusted value of the right Trigger 
     * @param scale input scale
     * @return adjusted value
     */
    public double getRightT(Scale scale) {
        return getOutput(getRightTriggerAxis() > Constants.controller.TRIGGER_THRESHOLD ? getRightTriggerAxis() : 0.0, scale);
    }

    /**
     * Get the adjusted value of the right Trigger
     * @param scale input scale
     * @return adjusted value
     */
    public double getLeftT(Scale scale) {
        return getOutput(getLeftTriggerAxis() > Constants.controller.TRIGGER_THRESHOLD ? getLeftTriggerAxis() : 0.0, scale);
    }

    /**
     * Vibrates the controller
     * @param intensity strength of vibration
     */
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
