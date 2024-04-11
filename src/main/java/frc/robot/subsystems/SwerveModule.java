package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final double TWO_PI = 2 * Math.PI;

    // getRotations
    private static double RadiansToRotations(final double radians) {
        return radians / TWO_PI;
    }
    
    private static double DegreesToRotations(final double degrees) {
        return RadiansToRotations(Math.toRadians(degrees)); 
    }
    
    // fromRotations
    private static double RotationsToRadians(final double rotations) {
        return rotations * TWO_PI; 
    }

    public static double RPMToMechanism(final double rpm, final double gear_ratio) {
        final double scalar = 2048.0 / 600.0;
        return (rpm * gear_ratio) * scalar; 
    }

    public static double MechanismToRPM(final double velocity_counts, final double gear_ratio) {
        final double scalar = 600.0 / 2048.0;
        double rpm = velocity_counts * scalar;        
        return rpm / gear_ratio;
    }

    public static double MechanismToMPS(final double velocity_counts, final double circumference, final double gear_ratio) {
        final double rpm = MechanismToRPM(velocity_counts, gear_ratio);
        return (rpm * circumference) / 60.0;
    }

    public static double MPSToMechanism(final double velocity, final double circumference, final double gear_ratio) {
        final double rpm = (velocity * 60.0) / circumference;
        return RPMToMechanism(rpm, gear_ratio);
    }

    private String m_module_name;
    private TalonFX m_drive;
    private TalonFX m_steer;
    private CoreCANcoder m_cancoder;
    private Rotation2d m_last_angle;

    private VelocityVoltage m_drive_velocity_voltage;
    private PositionVoltage m_steer_position_voltage;

    /**
     * Constructs a new SwerveModule object and initializes the
     * Drive & Steering Motors along with the CANcoder.
     * 
     * @param name A String representing the name of the module.
     *             Typically contains location of the module.
     * @param drive_id The can id of the drive motor. (Falcon 500)
     * @param steer_id The can id of the steer motor. (Falcon 500)
     * @param cancoder_id The can id of the CANcoder.
     */
    public SwerveModule(final String name, final int drive_id, final int steer_id, final int cancoder_id) {
        this.m_drive = new TalonFX(drive_id);
        this.m_steer = new TalonFX(steer_id);
        this.m_cancoder = new CANcoder(cancoder_id);
        this.m_module_name = name;

        // Get current configurations for all devices
        TalonFXConfiguration steer_config = new TalonFXConfiguration();
        this.m_steer.getConfigurator().refresh(steer_config);

        TalonFXConfiguration drive_config = new TalonFXConfiguration();
        this.m_drive.getConfigurator().refresh(drive_config);

        CANcoderConfiguration cancoder_config = new CANcoderConfiguration();
        this.m_cancoder.getConfigurator().refresh(cancoder_config);

        // Swerve CANCoder Configuration 
        cancoder_config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoder_config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        this.m_cancoder.getConfigurator().apply(cancoder_config);


        // Setup steering configuration
        TalonFXConfigurator steer_configurator = m_steer.getConfigurator();
        steer_config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steer_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steer_config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.STEER_ENABLE_CURRENT_LIMIT;
        steer_config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.STEER_CONTINUOUS_CURRENT_LIMIT;
        steer_config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.STEER_PEAK_CURRENT_LIMIT;
        steer_config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.STEER_PEAK_CURRENT_DURATION;
        steer_config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steer_config.Feedback.SensorToMechanismRatio = SwerveConstants.STEER_GEAR_RATIO;
        steer_config.Slot0.kP = SwerveConstants.STEER_KP;
        steer_config.Slot0.kI = SwerveConstants.STEER_KI;
        steer_config.Slot0.kD = SwerveConstants.STEER_KD;
        steer_config.HardwareLimitSwitch.ForwardLimitEnable = false;
        steer_config.HardwareLimitSwitch.ReverseLimitEnable = false;

        steer_configurator.apply(steer_config);


        // Setup drive configuration 
        TalonFXConfigurator drive_configurator = m_drive.getConfigurator();
        drive_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        drive_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        drive_config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
        drive_config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
        drive_config.CurrentLimits.SupplyCurrentThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        drive_config.CurrentLimits.SupplyTimeThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;
        drive_config.Feedback.SensorToMechanismRatio = SwerveConstants.SDSMK4_L1;
        drive_config.Slot0.kP = SwerveConstants.DRIVE_KP;
        drive_config.Slot0.kI = SwerveConstants.DRIVE_KI;
        drive_config.Slot0.kD = SwerveConstants.DRIVE_KD;
        drive_config.HardwareLimitSwitch.ForwardLimitEnable = false;
        drive_config.HardwareLimitSwitch.ReverseLimitEnable = false;

        drive_configurator.setPosition(0.0);
        
        drive_configurator.apply(drive_config);

        this.m_last_angle = this.GetState().angle;

        this.ResetToAbsolute();        

    }

    public void SetDriveBrake() {
        this.SetDriveBrake(true);
    }

    public void SetDriveBrake(boolean on) {
        this.m_drive.setNeutralMode(on ? NeutralModeValue.Brake : NeutralModeValue.Coast); 
    }

    /**
     * Convenience method to stop all output of both the drive
     * and steering motor. Typically called by 
     * SwerveDrive::Stop().
     */
    public void Stop() {
        this.m_drive.set(0.0);
        this.m_steer.set(0.0);
    }

    /**
     * Sets the desired speed (drive) and angle (steer) of the swerve module.
     * 
     * @param desired_state The desired state of the module.
     * @param is_open_loop Determines if the drive motor is set directly or 
     *                     by velocity.
     */
    public void SetDesiredState(SwerveModuleState desired_state, boolean is_open_loop) {
        // For steering, the swerve module should never have to change the angle of the wheel more than
        // 90 degrees to achieve the target angle.
        // Modify the desired_state's angle to be the shortest angle between the current and target angle.
        // Reverse direction of drive motor if necessary.
        double rotations = this.m_steer.getPosition().getValueAsDouble();
        desired_state = SwerveModuleState.optimize(desired_state, Rotation2d.fromRotations(rotations));

        this.SetAngle(desired_state);
        this.SetSpeed(desired_state, is_open_loop);
    }

    /**
     * Sets the desired speed (drive) of the swerve module.
     * 
     * @param desired_state The desired state of the module.
     * @param is_open_loop Determines if the drive motor is set directly or 
     *                     by velocity.
     */
    public void SetSpeed(SwerveModuleState desired_state, boolean is_open_loop) {
        if (is_open_loop) {
            double percent_output = desired_state.speedMetersPerSecond / SwerveConstants.DRIVE_MAX_SPEED;
            m_drive.set(percent_output);
        } else {
            double velocity = MPSToMechanism(
                    desired_state.speedMetersPerSecond,
                    SwerveConstants.DRIVE_WHEEL_CIRCUMFERENCE,
                    SwerveConstants.DRIVE_GEAR_RATIO);
            //double velocity = desired_state.speed.value() / DRIVE_WHEEL_CIRCUMFERENCE;

            if (velocity == 0) { 
                this.m_drive.setVoltage(0); 
            } else {
                VelocityVoltage velocityVoltage = this.m_drive_velocity_voltage.withVelocity(velocity);
                this.m_drive.setControl(velocityVoltage);
            }
        }

    }

    /**
     * Sets the desired angle (steer) of the swerve module.
     * 
     * @param desired_state The desired state of the module.
     */
    public void SetAngle(SwerveModuleState desired_state) {
        //frc::Rotation2d angle = (fabs(desired_state.speed.value()) <= (DRIVE_MAX_SPEED * 0.005)) ? m_last_angle : desired_state.angle; 
//        m_steer->SetControl(m_steer_position_voltage.WithPosition(units::angle::turn_t(RadiansToRotations(angle.Radians().value()))));
//        m_last_angle = angle;

        double rotations = desired_state.angle.getRotations();
        PositionVoltage positionVoltage = this.m_steer_position_voltage.withPosition(rotations);
        this.m_steer.setControl(positionVoltage);
    }

    /**
     * Sets the desired angle (steer) of the swerve module.
     * 
     * @param angle The desired state of the module.
     */
    public void SetAngle(Rotation2d angle) {
        double rotations = angle.getRotations();
        PositionVoltage positionVoltage = this.m_steer_position_voltage.withPosition(rotations);
        this.m_steer.setControl(positionVoltage);
//        m_last_angle = frc::Rotation2d(units::radian_t(absAngle));
        this.m_last_angle = angle;
    }

    /**
     * Gets the current angle (steer) of the swerve module.
     * The angle returned is from the internal steer motor's
     * Rotor Sensor.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */
    public Rotation2d GetAngle() {
        double rotations = this.m_steer.getPosition().getValueAsDouble();
        return Rotation2d.fromRotations(rotations);
    }
    
    /**
     * Gets the current angle (steer) of the swerve module to
     * be used for Odometry.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */
    public Rotation2d GetAngleForOdometry() {
        double rotations = this.m_steer.getPosition().getValueAsDouble() % 1.0;
        return Rotation2d.fromRotations(rotations);
    }

    /**
     * Gets the current CANcoder angle (steer) of the swerve module.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */
    public Rotation2d GetCanCoder() {
        double rotations = this.m_cancoder.getAbsolutePosition().getValueAsDouble();
        return Rotation2d.fromRotations(rotations);
    }

    /**
     * Sets the steers motor's rotor sensor to match the current
     * CANcoder value. (Typically called by SwerveDrive::ResetToAbsolute())
     */
    public void ResetToAbsolute() {
        double position = this.m_cancoder.getAbsolutePosition().getValueAsDouble();
        this.m_steer.setPosition(-position);//RadiansToRotations(GetCanCoder().Radians().value())));
        //SetAngle(frc::Rotation2d(units::radian_t(0)));
     }

    /**
     * Gets the module's current state. (Speed and Angle)
     * 
     * @return SwerveModuleState Object representing the speed
     *         and angle of the swerve module.
     */
    public SwerveModuleState GetState() {
        double speedMetersPerSecond = MechanismToMPS(
            this.m_drive.getPosition().getValueAsDouble(),
            SwerveConstants.DRIVE_WHEEL_CIRCUMFERENCE,
            SwerveConstants.DRIVE_GEAR_RATIO);
        return  new SwerveModuleState(speedMetersPerSecond, this.GetAngle());
    }

    /**
     * Gets the module's current position.
     * 
     * @return SwerveModulePosition Object representing the distance
     *                              the drive module has traveled and
     *                              the angle of the swerve module.
     */
    public SwerveModulePosition GetPosition() {
        double distanceMeters = m_drive.getPosition().getValueAsDouble() * SwerveConstants.DRIVE_WHEEL_CIRCUMFERENCE;
        return new SwerveModulePosition(distanceMeters, this.GetAngle());
    }

    /**
     * Convenience method providing a location to output
     * telemetry to the SmartDashboard. This method is
     * called in SwerveDrive::PutTelemetry().
     */
    public void PutTelemetry() {
        SmartDashboard.putNumber(String.format("%s CC Pos", this.m_module_name), this.m_cancoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(String.format("%s Pos", this.m_module_name), this.m_steer.getPosition().getValueAsDouble());
    }
}
