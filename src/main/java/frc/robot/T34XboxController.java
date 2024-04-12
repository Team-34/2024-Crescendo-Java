package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class T34XboxController extends XboxController {
    public enum AxisType {
        TRIGGER,
        X_AXIS,
        Y_AXIS,
        Z_AXIS,
    }

    public enum JoystickHand {
    	LEFT,
    	RIGHT,
    }

    private double m_left_x_db        = 0;
    private double m_left_y_db        = 0;
    private double m_right_x_db       = 0;
    private double m_right_y_db       = 0;
    private double m_left_trigger_db  = 0;
    private double m_right_trigger_db = 0;

    public T34XboxController(final int port) {
        super(port);
    }

    public T34XboxController(final T34XboxController other) {
        super(other.getPort());
    }

    public void setAllAxisDeadband(final double deadband) {
        if (0.0 <= deadband && deadband < 1.0) {
            this.m_left_x_db        = deadband;
            this.m_left_y_db        = deadband;
            this.m_right_x_db       = deadband;
            this.m_right_y_db       = deadband;
            this.m_left_trigger_db  = deadband;
            this.m_right_trigger_db = deadband;
        }
    }

    public void setAxisDeadband(final JoystickHand hand, final AxisType axis, final double value) {
        final double clampedValue = Maths.clamp(value, 0.0, 1.0);

        switch (axis)
        {
            case TRIGGER:
                if (hand == JoystickHand.LEFT)
                    this.m_left_trigger_db = clampedValue;
                else 
                    this.m_right_trigger_db = clampedValue;
                break;
            case X_AXIS:
                if (hand == JoystickHand.LEFT)
                    this.m_left_x_db = clampedValue;
                else
                    this.m_right_x_db = clampedValue;
                break;
            case Y_AXIS:
                if (hand == JoystickHand.LEFT)
                    this.m_left_y_db = clampedValue;
                else    
                    this.m_right_y_db = clampedValue;
                break;
            default:
                break;
        }
    }

    public double getXDB(final JoystickHand hand) {
        double value = hand == JoystickHand.LEFT ? this.getLeftX() : this.getRightX();
        final double db = hand == JoystickHand.LEFT ? this.m_left_x_db : this.m_right_x_db;

        if (value < 0.0) {
            if (value > -db) {
                value = 0.0;
            }
        } else if (value < db) {
            value = 0.0;
        }

        return value;
    }

    public double getYDB(final JoystickHand hand) {
        double value = hand == JoystickHand.LEFT ? this.getLeftY() : this.getRightY();
        final double db = hand == JoystickHand.LEFT ? this.m_left_y_db : this.m_right_y_db;

        if (value < 0.0) {
            if (value > -db) {
                value = 0.0;
            }
        } else if (value < db) {
            value = 0.0;
        }

        return value;
    }

    public double getTriggerDB(JoystickHand hand) {
        double value = hand == JoystickHand.LEFT ? this.getLeftTriggerAxis() : this.getRightTriggerAxis();
        final double db = hand == JoystickHand.LEFT ? this.m_left_trigger_db : this.m_right_trigger_db;

        if (value < 0.0) {
            if (value > -db) {
                value = 0.0;
            }
        } else if (value < db) {
            value = 0.0;
        }

        return value;
    }

    public double getTriggersCoercedDB() {
        return this.getLeftTriggerDB() + -this.getRightTriggerDB();
    }
    	
    public double getLeftStickXDB() {
        return this.getXDB(JoystickHand.LEFT);
    }
    
    public double getLeftStickYDB() {
        return this.getYDB(JoystickHand.LEFT);
    }

    public double getRightStickXDB() {
        return this.getXDB(JoystickHand.RIGHT);
    }
    
    public double getRightStickYDB() {
        return this.getYDB(JoystickHand.RIGHT);
    }

    public double getLeftTriggerDB() {
        return this.getTriggerDB(JoystickHand.LEFT);
    }

    public double getRightTriggerDB() {
        return this.getTriggerDB(JoystickHand.RIGHT);
    }
}
