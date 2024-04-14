// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ControllerDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightUtil;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

import java.time.Clock;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private Clock clock = null;
    public T34XboxController ctrl = null;
    public SwerveDrive swerve_drive = null;

    public Shooter shooter = null;
    // public Climber climber = null;
    public TrajMath traj_math = null;
    public LimelightUtil limelight_util = null;

    public double arm_angle_setpoint;

    public double auto_start_dist_1;
    public double auto_start_dist_2;
    public double auto_end_dist_1;
    public double auto_end_dist_2;
    public double auto_current_dist;
    public boolean auto_finished_driving_1;
    public boolean auto_finished_driving_2;
    public boolean auto_finished_aiming;
    public boolean auto_finished_shooting;

    public SendableChooser<String> path_chooser = null;

    public ControllerDriveCommand DefaultCommand = null;


    // // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // // Replace with CommandPS4Controller or CommandJoystick if needed
    // private final CommandXboxController m_driverController =
    //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        this.clock = Clock.systemUTC();
        this.swerve_drive = new SwerveDrive();
        this.ctrl = new T34XboxController(0);
        this.shooter = new Shooter(this.clock);
        // this.climber = new Climber();
        this.traj_math = new TrajMath(
            23.884,
            1.9815,
            1.435,
            0.2688,
            ((this.shooter.getTopArmEncoderVal() + this.shooter.getBottomArmEncoderVal()) * 0.5) / Constants.ARM_DEG_SCALAR,
            30.0
        );
        this.limelight_util = new LimelightUtil(
            this.traj_math,
            LimelightUtil.TargetMode.kSpeaker
        );
        this.arm_angle_setpoint = 90.0;
        this.auto_start_dist_1 = 0.0;
        this.auto_end_dist_1 = 0.0;
        this.auto_start_dist_2 = 0.0;
        this.auto_end_dist_2 = 0.0;
        this.auto_finished_driving_1 = false;
        this.auto_finished_aiming = true;
        this.auto_finished_driving_2 = true;
        this.auto_finished_shooting = true;
        this.DefaultCommand = new ControllerDriveCommand(this.swerve_drive, this.ctrl, this.clock);
        
        this.ctrl.setAllAxisDeadband(0.2);
        // Configure the trigger bindings
        this.configureBindings();

        this.path_chooser.setDefaultOption("None", "None");
        this.path_chooser.addOption("Faris room path", "TestingPathFarisRoom");
    
        this.path_chooser.addOption("Start at left, score 2 points", "Left_Score1_NoAmp_MoveOut");
        this.path_chooser.addOption("Start at left, score 3 points", "Left_Score2_NoAmp_MoveOut");
        this.path_chooser.addOption("Start at left, score 4 points", "Left_Score3_NoAmp_MoveOut");
    
        this.path_chooser.addOption("Start at mid, score 2 points", "Mid_Score1_NoAmp_MoveOut");
        this.path_chooser.addOption("Start at mid, score 3 points", "Mid_Score2_NoAmp_MoveOut");
        this.path_chooser.addOption("Start at mid, score 4 points", "Mid_Score3_NoAmp_MoveOut");
    
        this.path_chooser.addOption("Start at right, score 2 points", "Right_Score1_NoAmp_MoveOut");
        this.path_chooser.addOption("Start at right, score 3 points", "Right_Score2_NoAmp_MoveOut");
        this.path_chooser.addOption("Start at right, score 4 points", "Right_Score3_NoAmp_MoveOut");
    
        this.path_chooser.addOption("Start at left, score in amp", "Left_Score0_YesAmp_MoveAuto");
        this.path_chooser.addOption("Start at mid, score in amp", "Middle_Score0_YesAmp_MoveAuto");
        this.path_chooser.addOption("Start at left, score in amp", "Right_Score0_YesAmp_MoveAuto");
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
      // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
      // new Trigger(m_exampleSubsystem::exampleCondition)
      //     .onTrue(new ExampleCommand(m_exampleSubsystem));

      // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
      // // cancelling on release.
      // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);

        return new InstantCommand(() -> {
          this.swerve_drive.drive(new Translation2d(0.0, -1.0), 0.0);
        }, this.swerve_drive);

        // final String path_file_name = this.path_chooser.getSelected();
        // final PathPlannerPath path = PathPlannerPath.fromPathFile(path_file_name);
    
        // return AutoBuilder.followPath(path);
    }
}
