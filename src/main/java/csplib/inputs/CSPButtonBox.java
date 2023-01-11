package csplib.inputs;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class CSPButtonBox extends Joystick{
    public CSPButtonBox(int port) {
        super(port);
    }

    public JoystickButton getButtonOneObj() {
        return new JoystickButton(this, 1);
    }

    public JoystickButton getButtonTwoObj() {
        return new JoystickButton(this, 2);
    }

    public JoystickButton getButtonThreeObj() {
        return new JoystickButton(this, 3);
    }

    public JoystickButton getButtonFourObj() {
        return new JoystickButton(this, 4);
    }

    public JoystickButton getButtonFiveObj() {
        return new JoystickButton(this, 5);
    }

    public JoystickButton getButtonSixObj() {
        return new JoystickButton(this, 6);
    }

    public JoystickButton getButtonSevenObj() {
        return new JoystickButton(this, 7);
    }

    public JoystickButton getButtonEightObj() {
        return new JoystickButton(this, 8);
    }

    public JoystickButton getButtonNineObj() {
        return new JoystickButton(this, 9);
    }
}
