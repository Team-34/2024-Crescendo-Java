// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private boolean bypass = false;


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
      // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
      // autonomous chooser on the dashboard.
      this.m_robotContainer = new RobotContainer();

      this.m_robotContainer.shooter.init();
      // this.m_robotContainer.climber.init();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        //static std::shared_ptr<t34::SwerveDrive> drive = m_this.m_robotContainer.SwerveDrive;
        final Gyro gyro = Gyro.get();
        
        SmartDashboard.putNumber("_Yaw", gyro.getAngle());
    
        this.m_robotContainer.swerve_drive.putTelemetry();
        this.m_robotContainer.shooter.putTelemetry();
    
        //Periodics
        this.m_robotContainer.shooter.periodic();
        // this.m_robotContainer.climber.periodic();
        this.m_robotContainer.limelight_util.periodic();
        //_________________________
    
        // Swerve Auto Drive Outputs
        SmartDashboard.putNumber("Auto drive x: ", this.m_robotContainer.limelight_util.m_swerve_drive_speeds.x);
        SmartDashboard.putNumber("Auto drive y: ", this.m_robotContainer.limelight_util.m_swerve_drive_speeds.y);
        SmartDashboard.putNumber("Auto drive r: ", this.m_robotContainer.limelight_util.m_swerve_drive_speeds.r);
        //_________________________
    
    
        // Misc.
        SmartDashboard.putNumber("Target ID: ", this.m_robotContainer.limelight_util.getTargetID());
        SmartDashboard.putNumber("Distance from limelight target (meters): ", this.m_robotContainer.limelight_util.m_math_handler.getDistanceFromTarget());
        SmartDashboard.putBoolean("Note Sensor detection", this.m_robotContainer.shooter.intakeHasNote());
        SmartDashboard.putBoolean("Arm Sensor detection", this.m_robotContainer.shooter.isArmAtZero());
    
        SmartDashboard.putNumber("Raw Arm Encoder Val: ", this.m_robotContainer.shooter.getTopArmEncoderVal() / Constants.ARM_ENC_CONVERSION_FACTOR);
    
        //_________________________
        /*
        //checks if the approx. range is nearing 5 meters (the range the LL with pick up an AT before the resolution becomes too low)
        if (log2( (2.5 / LimelightHelpers::getTA()) - 0.3) >= 5.0) {
            SmartDashboard.putNumber("Over maximum detection range, current distance is: ", this.m_robotContainer.limelight_util.m_math_handler.GetDistanceFromTarget());
        }
        //_________________________*/
        
        
        SmartDashboard.putData("Auto chooser: ", this.m_robotContainer.path_chooser);
    
        this.m_robotContainer.limelight_util.m_math_handler.putTelemetry();      
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        Gyro.get().zeroYaw();

        m_autonomousCommand = this.m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        this.m_robotContainer.shooter.setTolerance(0.03);
        this.m_robotContainer.shooter.setkP(0.5);

        CommandScheduler.getInstance().setDefaultCommand(
            this.m_robotContainer.swerve_drive,
            this.m_robotContainer.DefaultCommand);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        final Gyro gyro = Gyro.get();
    
        this.m_robotContainer.traj_math.inputMotorOutputPercent(this.m_robotContainer.shooter.getMaxSpeedPercent());
    
        // PROCESS CONTROLLER BUTTONS
        // Buttons are implemented this way out of simplicity.
        // Consider using button trigger events with commands instead.
    
        // Assign Back Button to Faris Mode.
        if (this.m_robotContainer.ctrl.getBackButtonReleased())
        {
            this.m_robotContainer.swerve_drive.toggleFarisMode();
        }
    
        // Assign Start Button to Zeroing Yaw.
        // Note: This is for emergency use only!
        // The robot should be oriented with the front pointed 
        // at the opposite end of the field and sides as 
        // parallel as possible to the fields sides when this
        // button is pressed/released.
        if (this.m_robotContainer.ctrl.getStartButtonReleased()) {
            gyro.zeroYaw();
        }
    
        // toggle PID vs basic motor output arm movement with the A button
        if (this.m_robotContainer.ctrl.getYButtonReleased()) { 
            this.m_robotContainer.arm_angle_setpoint = ((this.m_robotContainer.shooter.getTopArmEncoderVal() + this.m_robotContainer.shooter.getBottomArmEncoderVal()) * 0.5) / Constants.ARM_DEG_SCALAR;
            this.m_robotContainer.shooter.togglePIDArmMovement();
        }
    
        //if (this.m_robotContainer.ctrl.getXButton() == false && (this.m_robotContainer.ctrl.getRightTriggerAxis() < this.m_robotContainer.ctrl.getRightTriggerDB()))
        //{
        //    this.m_robotContainer.shooter.runIntakeMotorPercent(0.0);
        //}
    
        //Run the shooter with the triggers
          //Right is forward, left is back
        if (this.m_robotContainer.ctrl.getLeftTriggerAxis() > 0.2)
        {
            this.bypass = false;
            this.m_robotContainer.shooter.runShooterPercent(-(this.m_robotContainer.ctrl.getLeftTriggerAxis()));
        }
        else if (this.m_robotContainer.ctrl.getRightTriggerAxis() > 0.2)
        {
            this.bypass = true;
            this.m_robotContainer.shooter.runShooterPercent(this.m_robotContainer.ctrl.getRightTriggerAxis());
            //this.m_robotContainer.shooter.shoot(this.m_robotContainer.ctrl.getRightTriggerAxis());
        }
        else
        {
            this.bypass = false;
            this.m_robotContainer.shooter.runShooterPercent(0.0);
        }
    
        //Set the robot's target mode with the D-Pad
        switch (this.m_robotContainer.ctrl.getPOV())
        {
            case (Constants.POV_UP): //  rest
                this.m_robotContainer.shooter.configForRest();
                
                break;
            case (Constants.POV_RIGHT): // amp
                this.m_robotContainer.limelight_util.targetAmp();
                this.m_robotContainer.shooter.configForAmp();
                break;
            case (Constants.POV_DOWN): // note collection
                
                this.m_robotContainer.shooter.configForNoteCollection();
                break;
            case (Constants.POV_LEFT): // max speed
                this.m_robotContainer.limelight_util.targetSpeaker();
                this.m_robotContainer.shooter.configForSpeaker(this.m_robotContainer.traj_math.getArmFiringAngleDeg());
                break;
        }
    
        //Move the arm with the bumpers
          //Right bumper increases angle, left bumper decreases angle
    
        if (this.m_robotContainer.ctrl.getLeftBumper() && this.m_robotContainer.shooter.usingPIDArmMovement() && this.m_robotContainer.shooter.isArmAtZero() == false)
        {
            this.m_robotContainer.shooter.moveDown();
        }
        else if (this.m_robotContainer.ctrl.getRightBumper() && this.m_robotContainer.shooter.usingPIDArmMovement() == false)
        {
            this.m_robotContainer.shooter.runTopArmMotorPercent(0.25);
            this.m_robotContainer.shooter.runBottomArmMotorPercent(0.25);
        }
        else if (this.m_robotContainer.ctrl.getRightBumper() && this.m_robotContainer.shooter.usingPIDArmMovement())
        {
            this.m_robotContainer.shooter.moveUp();
        }
        else if (this.m_robotContainer.ctrl.getLeftBumper() && this.m_robotContainer.shooter.usingPIDArmMovement() == false && this.m_robotContainer.shooter.isArmAtZero() == false)
        {
            this.m_robotContainer.shooter.runTopArmMotorPercent(-0.3);
            this.m_robotContainer.shooter.runBottomArmMotorPercent(-0.3);
        }
        else if (this.m_robotContainer.shooter.usingPIDArmMovement() == false)
        {
            this.m_robotContainer.shooter.runTopArmMotorPercent(0.0);
            this.m_robotContainer.shooter.runBottomArmMotorPercent(0.0);
        }
    
        //Run intake backward with the X button, forward with A button
        if (this.m_robotContainer.ctrl.getXButton())
        {
            this.m_robotContainer.shooter.runIntakeMotorPercent(-0.7);
        }
        else if (this.m_robotContainer.ctrl.getAButton())
        {
            this.m_robotContainer.shooter.runIntakeMotorPercent(0.7, this.bypass);
        }
        else
        {
            if (!this.bypass)
                this.m_robotContainer.shooter.runIntakeMotorPercent(0.0, this.bypass);
        }
    
        //if (this.m_robotContainer.ctrl.getYButton()) // run swerve automatically using the limelight with the Y button
        //{
        //  frc2::Command([this]
        //  {
        //      this.m_robotContainer.swerve_drive->Drive(
        //          frc::Translation2d(
        //              units::meter_t(this.m_robotContainer.limelight_util.m_swerve_drive_speeds.x),
        //              units::meter_t(this.m_robotContainer.limelight_util.m_swerve_drive_speeds.y)
        //              ), this.m_robotContainer.limelight_util.m_swerve_drive_speeds.r
        //      );
    //
        //  }).Schedule();
        //}
    
        this.m_robotContainer.shooter.periodic();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
