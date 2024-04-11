package frc.robot.subsystems;

import java.util.Arrays;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gyro;
import frc.robot.Maths;

public class SwerveDrive extends SubsystemBase {
        private boolean m_field_oriented = true;
        private boolean m_faris_mode = false;
        private double m_speed_scalar = SwerveConstants.FARIS_SPEED_MODE_SCALAR;

        private MedianFilter m_filter = null;
        private Gyro m_gyro = null;
        private SwerveModule[] m_swerve_modules = null;
        private SwerveDriveKinematics m_swerve_drive_kinematics = null;
        private SwerveDriveOdometry m_swerve_odometry = null;

    public SwerveDrive() {
        this.setName("SwerveDrive");

        this.m_gyro = Gyro.get();
        this.m_gyro.reset();

        this.m_filter = new MedianFilter(5);

        this.m_swerve_modules = new SwerveModule[] {
            new SwerveModule("SM_L_FWD", SwerveConstants.ID_LEFT_FWD_DRIVE,  SwerveConstants.ID_LEFT_FWD_STEER,  SwerveConstants.ID_LEFT_FWD_CANCODER),   //,  frc::Rotation2d(units::radian_t(RotationsToRadians(LEFT_FWD_CANCODER_OFFSET)))),
            new SwerveModule("SM_R_FWD", SwerveConstants.ID_RIGHT_FWD_DRIVE, SwerveConstants.ID_RIGHT_FWD_STEER, SwerveConstants.ID_RIGHT_FWD_CANCODER),  //, frc::Rotation2d(units::radian_t(RotationsToRadians(RIGHT_FWD_CANCODER_OFFSET)))),
            new SwerveModule("SM_L_AFT", SwerveConstants.ID_LEFT_AFT_DRIVE,  SwerveConstants.ID_LEFT_AFT_STEER,  SwerveConstants.ID_LEFT_AFT_CANCODER),   //,  frc::Rotation2d(units::radian_t(RotationsToRadians(LEFT_AFT_CANCODER_OFFSET)))),
            new SwerveModule("SM_R_AFT", SwerveConstants.ID_RIGHT_AFT_DRIVE, SwerveConstants.ID_RIGHT_AFT_STEER, SwerveConstants.ID_RIGHT_AFT_CANCODER)   //, frc::Rotation2d(units::radian_t(RotationsToRadians(RIGHT_AFT_CANCODER_OFFSET)))),
        };

        this.m_swerve_drive_kinematics = new SwerveDriveKinematics(
            // Translation2d objects assumes the robot is at the origin facing in the positive X direction, 
            // forward is positive X and left is positive Y.
            new Translation2d( SwerveConstants.SWERVE_MODULE_FROM_CENTER,  SwerveConstants.SWERVE_MODULE_FROM_CENTER), // Left Forward Module
            new Translation2d( SwerveConstants.SWERVE_MODULE_FROM_CENTER, -SwerveConstants.SWERVE_MODULE_FROM_CENTER), // Right Forward Module
            new Translation2d(-SwerveConstants.SWERVE_MODULE_FROM_CENTER,  SwerveConstants.SWERVE_MODULE_FROM_CENTER), // Left Aft Module
            new Translation2d(-SwerveConstants.SWERVE_MODULE_FROM_CENTER, -SwerveConstants.SWERVE_MODULE_FROM_CENTER)  // Right Aft Module                                                                  
        );

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(this.m_swerve_modules[0].getPosition().distanceMeters, this.m_swerve_modules[0].getCanCoder()),
            new SwerveModulePosition(this.m_swerve_modules[1].getPosition().distanceMeters, this.m_swerve_modules[1].getCanCoder()),
            new SwerveModulePosition(this.m_swerve_modules[2].getPosition().distanceMeters, this.m_swerve_modules[2].getCanCoder()),
            new SwerveModulePosition(this.m_swerve_modules[3].getPosition().distanceMeters, this.m_swerve_modules[3].getCanCoder()),
        };
        this.m_swerve_odometry = new SwerveDriveOdometry(
            this.m_swerve_drive_kinematics,
            Rotation2d.fromDegrees(this.m_gyro.getAngle()),
            modulePositions);

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
            this::GetPose, // Robot pose supplier
            this::ResetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::GetRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::DriveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD), // Translation PID constants
                new PIDConstants(SwerveConstants.STEER_KP, SwerveConstants.STEER_KI, SwerveConstants.STEER_KD), // Rotation PID constants
                SwerveConstants.DRIVE_MAX_SPEED, // Max module speed, in m/s
                SwerveConstants.SWERVE_MODULE_FROM_CENTER, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Toggles Faris speed mode on or off. When on, drive outputs
     * will be scaled by a percentage. This percentage is hard
     * coded into FARIS_SPEED_MODE_SCALAR constexpr located in
     * SwerveConstants.h and should be a value between 0.1 and 1.0. 
     */
    public void ToggleFarisMode() {
        this.m_faris_mode = !this.m_faris_mode;
        if (this.m_faris_mode) {
            this.m_speed_scalar = Maths.clamp(SwerveConstants.FARIS_SPEED_MODE_SCALAR, 0.1, 1.0);
        } else {
            this.m_speed_scalar = 1.0;
        }

    }

    public void Drive(Translation2d translation, double rotation) {
        this.Drive(translation, rotation, true);
    }

    public void Drive(Translation2d translation, double rotation, boolean field_relative) {
        this.Drive(translation, rotation, field_relative, false);
    }

    public void Drive(Translation2d translation, double rotation, boolean field_relative, boolean is_open_loop) {
        this.m_field_oriented = field_relative;

        ChassisSpeeds speeds = null;
        if (field_relative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                (translation.getY() / SwerveConstants.DRIVE_MAX_SPEED) * m_speed_scalar, 
                (translation.getX() / SwerveConstants.DRIVE_MAX_SPEED) * m_speed_scalar, 
                (rotation / SwerveConstants.STEER_MAX_SPEED) * m_speed_scalar, 
                Rotation2d.fromDegrees(this.m_gyro.getYaw() * -1.0) // SHOULD THIS BE INVERTED????
            );
        } else {
            // PROBABLY NEED TO SWAP X & Y: NOT YET TESTED
            speeds = new ChassisSpeeds(
                -translation.getX() / SwerveConstants.DRIVE_MAX_SPEED,
                translation.getY() / SwerveConstants.DRIVE_MAX_SPEED,
                rotation);
        }

        var sms = this.m_swerve_drive_kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(sms, SwerveConstants.DRIVE_MAX_SPEED);

        for (int i = 0; i < this.m_swerve_modules.length; ++i) {
            m_swerve_modules[i].setDesiredState(sms[i], is_open_loop);
        }
    }

    public void DriveRobotRelative(ChassisSpeeds speeds) {
        this.DriveAuto(speeds);
    }

    public void DriveAuto(ChassisSpeeds speeds) {
        //units::meters_per_second_t temp_vx{speeds.vx};
        //
        //speeds.vx = -speeds.vy;
        //speeds.vy = temp_vx;

        speeds.omegaRadiansPerSecond *= -1.0;

        var sms = m_swerve_drive_kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(sms, SwerveConstants.DRIVE_MAX_SPEED);

        for (int i = 0; i < this.m_swerve_modules.length; ++i) {
            this.m_swerve_modules[i].setDesiredState(sms[i], false);
        }
    }

    /**
     * Convenience method to stop all output of
     * both the drive and steering motor for all swerve
     * modules. 
     */
    public void Stop() {
        for (var sm : this.m_swerve_modules) sm.stop();
    }


    public ChassisSpeeds GetRobotRelativeSpeeds() {
        return this.m_swerve_drive_kinematics.toChassisSpeeds(this.GetModuleStates());
    }

    /**
     * Get the current pose of the robot.
     * 
     * @return Pose2d Object that contains translational and rotational elements.
     */
    public Pose2d GetPose() {
        return this.m_swerve_odometry.getPoseMeters();
    }

    /**
     * Get the current pose's transitional x component of the robot.
     * 
     * @return double Represent the pose's transitional x component.
     */    
    public double GetPoseX() {
        return this.GetPose().getX();
    }

    /**
     * Get the current pose's transitional y component of the robot.
     * 
     * @return double Represent the pose's transitional y component.
     */    
    public double GetPoseY() {
        return this.GetPose().getY();
    }

    /**
     * Get the current pose's rotational component of the robot.
     * 
     * @return double Represent the pose's rotational component in degrees.
     */
    public double GetPoseRotation() {
        return this.GetPose().getRotation().getDegrees();
    }

    public void ResetPose(Pose2d pose) {
        this.ResetOdometry(pose);
    }

    /**
     * Reset the odometer for all swerve modules.
     */
    public void ResetOdometry(Pose2d pose) {
        // this.m_swerve_odometry.resetPosition(frc::Rotation2d(units::degree_t(m_gyro->GetAngle())), GetModulePositions(), pose);
        this.m_swerve_odometry.resetPosition(
            Rotation2d.fromDegrees(this.m_gyro.getAngle()), 
            this.GetModulePositions(), 
            pose);
    }

    /**
     * Get an array of the swerve module states.
     * 
     * @return SwerveModuleState[4] Array of objects representing the speed
     *                              and angle of each swerve module.
     */
    public SwerveModuleState[] GetModuleStates() {
        // var states = new SwerveModuleState[this.m_swerve_modules.length];
        // for (int i = 0; i < this.m_swerve_modules.length; i++) {
        //     states[i] = this.m_swerve_modules[i].getState();
        // }
        // return states;

        // return Arrays.stream(this.m_swerve_modules)
        //     .map(x -> x.getState())
        //     .toArray(SwerveModuleState[]::new);

        return Arrays.stream(this.m_swerve_modules)
            .map(SwerveModule::getState)
            .toArray(SwerveModuleState[]::new);
    }

    /**
     * Get an array of the swerve positions.
     * 
     * @return SwerveModulePosition[4] Array of objects representing the distance
     *                                 traveled and the angle of each swerve module.
     */
    public SwerveModulePosition[] GetModulePositions() {
        // var positions = new SwerveModulePosition[this.m_swerve_modules.length];
        // for (int i = 0; i < this.m_swerve_modules.length; i++) {
        //     var module = this.m_swerve_modules[i];
        //     double distanceMeters = module.getPosition().distanceMeters;
        //     Rotation2d angle = module.getCanCoder();
        //     positions[i] = new SwerveModulePosition(distanceMeters, angle);
        // }
        // return positions;

        // return Arrays.stream(this.m_swerve_modules).map(module -> {
        //     final double distanceMeters = module.getPosition().distanceMeters;
        //     final Rotation2d angle = module.getCanCoder();
        //     return new SwerveModulePosition(distanceMeters, angle);
        // }).toArray(SwerveModulePosition[]::new);

        return Arrays.stream(this.m_swerve_modules).map(module -> {
            SwerveModulePosition position = module.getPosition();
            position.angle = module.getCanCoder();
            return position;
        }).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Get an array of the swerve positions.
     * 
     * @return SwerveModulePosition[4] Array of objects representing the inverted distance
     *                                 traveled and the angle of each swerve module.
     */
    public SwerveModulePosition[] GetModulePositionsInverted() {
        // var positions = new SwerveModulePosition[this.m_swerve_modules.length];
        // for (int i = 0; i < this.m_swerve_modules.length; i++) {
        //     var position = this.m_swerve_modules[i].getPosition();
        //     position.distanceMeters *= -1;
        //     positions[i] = position;
        // }
        // return positions;

        return Arrays.stream(this.m_swerve_modules).map(module -> {
            SwerveModulePosition position = module.getPosition();
            position.distanceMeters *= -1;
            return position;
        }).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Gets the yaw of the robot. A filter is used 
     * to lessen the impact of outliers in the gyro
     * data.
     * 
     * @return Rotation2d A rotation in a 2D coordinate frame.
     */    
    public Rotation2d GetYaw() {
        final double yawDegrees = Constants.INVERT_GYRO
            ? (360.0 - this.m_gyro.getYaw())
            : this.m_gyro.getYaw();
        return Rotation2d.fromDegrees(this.m_filter.calculate(yawDegrees));
    }

    /**
     * Sets the steers motor's rotor sensor to match the current
     * CANcoder value for each swerve module.
     */
    public void ResetToAbsolute() {
        for (var module : this.m_swerve_modules) module.resetToAbsolute();
    }

    /**
     * DO NOT CALL DIRCTLY.
     * This method is called periodically by the CommandScheduler.
     * It is used here to update the swerve module odometer.
     */     
    @Override
    public void periodic() {
        // this.m_swerve_odometry.update(
        //     this.GetYaw().unaryMinus(), 
        //     new SwerveModulePosition[] {
        //         new SwerveModulePosition( this.m_swerve_modules[0].getPosition().distanceMeters, this.m_swerve_modules[0].getCanCoder() ),
        //         new SwerveModulePosition( this.m_swerve_modules[1].getPosition().distanceMeters, this.m_swerve_modules[1].getCanCoder() ),
        //         new SwerveModulePosition( this.m_swerve_modules[2].getPosition().distanceMeters, this.m_swerve_modules[2].getCanCoder() ),
        //         new SwerveModulePosition( this.m_swerve_modules[3].getPosition().distanceMeters, this.m_swerve_modules[3].getCanCoder() )
        //     });

        this.m_swerve_odometry.update(this.GetYaw().unaryMinus(), this.GetModulePositions());
        
        SmartDashboard.putData(this); 
    }

    public void PutTelemetry() {
        SmartDashboard.putBoolean("Field Oriented", this.m_field_oriented);
        SmartDashboard.putBoolean("Faris Mode", this.m_faris_mode);

        for (var module : this.m_swerve_modules) module.putTelemetry();
    }
}
