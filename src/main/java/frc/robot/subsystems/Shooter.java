package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Maths;

public class Shooter {
    //double m_arm_angle_top{};
    //double m_arm_angle_bottom{};

    private double m_max_speed_percent  = 1.0;
    private double m_arm_angle_setpoint = 82.0;
    private double m_tolerance          = 1.0;
    private double m_kp                 = 0.0;

    private boolean arm_using_pid = true;

    private CANSparkMax m_firing_motor_left  = null;
    private CANSparkMax m_firing_motor_right = null;
    private CANSparkMax m_arm_motor_top      = null;
    private CANSparkMax m_arm_motor_bottom   = null;

    private RelativeEncoder m_arm_encoder_top    = null;
    private RelativeEncoder m_arm_encoder_bottom = null;

    private SparkPIDController m_arm_pidctrl_top    = null;
    private SparkPIDController m_arm_pidctrl_bottom = null;

    private TalonSRX m_intake_motor = null; // Only supported in Phoenix 5

    //     //frc::PIDController m_arm_pid;

    private DigitalInput m_note_sensor = null;
    private DigitalInput m_arm_sensor  = null;

    public Shooter() {
        this.m_firing_motor_left  = new CANSparkMax(14, MotorType.kBrushless);
        this.m_firing_motor_right = new CANSparkMax(11, MotorType.kBrushless);
        this.m_arm_motor_top      = new CANSparkMax(2, MotorType.kBrushless);
        this.m_arm_motor_bottom   = new CANSparkMax(1, MotorType.kBrushless);
        this.m_intake_motor       = new TalonSRX(12);
        this.m_arm_encoder_top    = this.m_arm_motor_top.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        this.m_arm_encoder_bottom = this.m_arm_motor_bottom.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        
        this.m_note_sensor = new DigitalInput(9);
        this.m_arm_sensor  = new DigitalInput(8);

        this.m_arm_pidctrl_top = this.m_arm_motor_top.getPIDController();
        this.m_arm_pidctrl_top.setP(0.5);
        this.m_arm_pidctrl_top.setI(0.0);
        this.m_arm_pidctrl_top.setD(0.05);

        this.m_arm_pidctrl_bottom = this.m_arm_motor_bottom.getPIDController();
        this.m_arm_pidctrl_bottom.setP(0.5);
        this.m_arm_pidctrl_bottom.setI(0.0);
        this.m_arm_pidctrl_bottom.setD(0.05);
    }

    public void runShooterPercent(final double motor_output) {
        m_firing_motor_left.set(Maths.clamp(motor_output, -this.m_max_speed_percent, this.m_max_speed_percent));
        m_firing_motor_right.set(Maths.clamp(motor_output, -this.m_max_speed_percent, this.m_max_speed_percent));
    
        //runIntakeMotorPercent(motor_output, true);
    }

    public void runIntakeMotorPercent(final double motor_output) {
        this.runIntakeMotorPercent(motor_output, false);
    }

    public void runIntakeMotorPercent(final double motor_output, final boolean bypass_sensor) {
        double clamp_max = (this.intakeHasNote() && !bypass_sensor) ? 0.0 : 1.0;
        double percent_output = Maths.clamp(motor_output, -1.0, clamp_max);
        this.m_intake_motor.set(ControlMode.PercentOutput, percent_output);
    }

    //public void MoveArmToAngleDeg(final double angle)
    //{
    //    m_arm_angle_setpoint = angle * ARM_DEG_SCALAR;
    //
    //    m_arm_pidctrl_top.SetReference(m_arm_angle_setpoint, rev::ControlType::kPosition);
    //    m_arm_pidctrl_bottom.SetReference(m_arm_angle_setpoint, rev::ControlType::kPosition);
    //    
    //}
    //
    //public void MoveShooterToAngleDeg(final double angle)
    //{
    //    m_arm_angle_setpoint = ((angle - SHOOTER_OFFSET_ANGLE_DEG) * ARM_DEG_SCALAR);
    //
    //    MoveArmToAngleDeg(angle);
    //}

    public double getMaxSpeedPercent() {
        return this.m_max_speed_percent;
    }
    public void setMaxSpeedPercent(final double percent) {
        this.m_max_speed_percent = percent;
    }

    public void configForAmp() {
        this.setSetpoint(87.18);
        this.setMaxSpeedPercent(0.1);
    }

    public void configForSpeaker(final double shooter_firing_angle) {
        this.setSetpoint(67.5);
        this.setMaxSpeedPercent(1.0);    
    }

    public void configForRest() {
        this.setSetpoint(90.0);
        this.setMaxSpeedPercent(1.0);    
    }

    public void configForNoteCollection() {
        this.setSetpoint(23.0);
        this.setMaxSpeedPercent(1.0);    
    }
    
    public void periodic() {
        this.m_arm_angle_setpoint = Maths.clamp(this.m_arm_angle_setpoint, 12.0, 90.0);

        //double motor_output = 
        //(fabs(m_kp * ( ( (m_arm_angle_setpoint / ARM_DEG_SCALAR) - GetTopArmEncoderVal()) / m_arm_angle_setpoint)) < m_tolerance) ? 
        //0.0 : (m_kp *  ( (m_arm_angle_setpoint / ARM_DEG_SCALAR) - GetTopArmEncoderVal()));
    
        if (this.usingPIDArmMovement())
        {
            //m_arm_motor_top.Set(motor_output);
            //m_arm_motor_bottom.Set(motor_output);
            final double rotations = this.m_arm_angle_setpoint * Constants.ARM_DEG_SCALAR;
            this.m_arm_pidctrl_top.setReference(rotations, ControlType.kPosition);
            this.m_arm_pidctrl_bottom.setReference(rotations, ControlType.kPosition);
        }
    }

    public void init() {
        this.m_arm_encoder_top.setPositionConversionFactor(Constants.ARM_ENC_CONVERSION_FACTOR);
        this.m_arm_encoder_bottom.setPositionConversionFactor(Constants.ARM_ENC_CONVERSION_FACTOR);

        this.m_arm_encoder_top.setPosition(2.4803999);
        this.m_arm_encoder_bottom.setPosition(this.m_arm_encoder_top.getPosition());

        this.m_arm_motor_top.setSmartCurrentLimit(20);
        this.m_arm_motor_bottom.setSmartCurrentLimit(20);

        this.m_firing_motor_left.setSmartCurrentLimit(20);
        this.m_firing_motor_right.setSmartCurrentLimit(20);    
    }

    public void putTelemetry() {
        SmartDashboard.putNumber("Arm Top Relative Encoder: ", this.getTopArmEncoderVal() / Constants.ARM_DEG_SCALAR);
        SmartDashboard.putNumber("Arm Bottom Relative Encoder: ", this.getBottomArmEncoderVal() / Constants.ARM_DEG_SCALAR);
        
        SmartDashboard.putNumber("Max Speed: ", m_max_speed_percent);
        SmartDashboard.putNumber("Arm Setpoint: ", m_arm_angle_setpoint);
    
        //SmartDashboard.putBoolean("IsIntakeMovingBackwards: ", IsIntakeMovingBackward(m_intake_motor.GetMotorOutputPercent()));
        SmartDashboard.putBoolean("UsingPIDArmMovement: ", this.usingPIDArmMovement());
    
    }

    public void setZero() {
        this.m_arm_encoder_top.setPosition(0.0);
        this.m_arm_encoder_bottom.setPosition(0.0);    
    }

    public void setSetpoint(final double setpoint) {
        this.m_arm_angle_setpoint = setpoint;
    }

    public void moveUp() { 
        this.m_arm_angle_setpoint += 0.5; 
    }
    public void moveDown() { 
        this.m_arm_angle_setpoint -= 0.5; 
    }

    public double getTopArmEncoderVal() {
        return this.m_arm_encoder_top.getPosition();
    }
    public double getBottomArmEncoderVal() { 
        return this.m_arm_encoder_bottom.getPosition(); 
    }

    public void runTopArmMotorPercent(final double motor_output) { 
        this.m_arm_motor_top.set(motor_output);
    }
    public void runBottomArmMotorPercent(final double motor_output) {
        this.m_arm_motor_bottom.set(motor_output); 
    }

    public void runLeftFiringMotorPercent(final double motor_output) { 
        this.m_firing_motor_left.set(motor_output); 
    }
    public void runRightFiringMotorPercent(final double motor_output) { 
        this.m_firing_motor_right.set(motor_output); 
    }

    public void togglePIDArmMovement() {
        this.arm_using_pid = !this.arm_using_pid;
    }
    public boolean usingPIDArmMovement() {
        return this.arm_using_pid;
    }

    public boolean intakeHasNote() {
        return this.m_note_sensor.get();
    }

    public boolean isArmAtZero() { 
        return m_arm_sensor.get(); 
    }

    public void setTolerance(final double t) {
        this.m_tolerance = t; 
    }

    public void setkP(final double kP) {
        this.m_kp = kP; 
    }

    private boolean isIntakeMovingBackward(final double motor_output) {
        return (-motor_output) < 0.0;
    }
}
