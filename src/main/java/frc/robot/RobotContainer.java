// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

//subsystem imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.IntakeShooter; 
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.operatorStuff;
import frc.robot.commands.DetectNoteHigh;
import frc.robot.commands.DetectNoteLow;
//import frc.robot.commands.DetectNote;
//import frc.robot.Constants.ClimberConstants;
//import frc.robot.commands.DetectNote;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetIntakeSpeed;
import frc.robot.commands.SetShooterSpeed;
// import frc.robot.commands.drivetrain.ResetRobotHeading;
// import frc.robot.commands.drivetrain.setXCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import  edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import edu.wpi.first.wpilibj.Joystick.AxisType;
// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.MathUtil;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
//import frc.robot.Constants.desiredEncoderValue;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer{
  // Subsystem initialization 
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser; 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Register Named Commands Path planner 
    registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();
     m_robotDrive.setCoast();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
              getDriverLeftY(),
              getDriverLeftX(),
              getDriverRightX(),
                true, true), // changed back to true 3/12/24 
            m_robotDrive));

            m_arm.setDefaultCommand(
            // Operator: left joystick arm control  
                new RunCommand(       
                () -> m_arm.setArm(operatorStuff.kArmSpeed*
                    MathUtil.applyDeadband(m_operatorController.getLeftY(),
                    ArmConstants.kArmDeadband)),m_arm));

  
            autoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public static CommandXboxController getDriverController() {
    return m_driverController;
  }

  public static CommandXboxController getOperatorController() {
    return m_operatorController;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

   public static double getLeftXWithDeadband() {
    return -MathUtil.applyDeadband(m_driverController.getLeftX(),
        OIConstants.kDriveDeadband);
  }

  public static double getDriverLeftX() {
    return .5 * Math.pow(getLeftXWithDeadband(), 5) + .5 * getLeftXWithDeadband();
  }

  public static double getLeftYWithDeadband() {
    return -MathUtil.applyDeadband(m_driverController.getLeftY(),
        OIConstants.kDriveDeadband);
  }

  public static double getDriverLeftY() {
    return .5 * Math.pow(getLeftYWithDeadband(), 5) + .5 * getLeftYWithDeadband();
  }

  public static double getRightXWithDeadband() {
    return -MathUtil.applyDeadband(m_driverController.getRightX(),
        OIConstants.kDriveDeadband);
  }

  public static double getDriverRightX() {
    return .5 * Math.pow(getRightXWithDeadband(), 5) + .5 * getRightXWithDeadband();
  }

  
  private void configureButtonBindings() {
    // operator controls 
    /*
     * Start Button: 
     * A Button: intake in
     * B Button: intake reverse  
     * X Button: shoot out 
     * Y Button: shoot reverse 
     * Start Button: shooter set up 
     */
        m_operatorController.x()
        .whileTrue(new SetShooterSpeed(m_shooter, Constants.IntakeShooter.kTopShootSpeed, Constants.IntakeShooter.kBottomShootSpeed))
        .whileFalse(new SetShooterSpeed(m_shooter, 0, 0));

        m_operatorController.y()
        .whileTrue(new SetShooterSpeed(m_shooter, -(Constants.IntakeShooter.kTopShootSpeed), -(Constants.IntakeShooter.kBottomShootSpeed)))
        .whileFalse(new SetShooterSpeed(m_shooter, 0, 0));

        m_operatorController.a()
        .whileTrue(new SetIntakeSpeed(m_intake, IntakeShooter.kIntakeSpeed))
        .whileFalse(new SetIntakeSpeed(m_intake, 0));

        m_operatorController.b()
        .whileTrue(new SetIntakeSpeed(m_intake, -(IntakeShooter.kIntakeSpeed)))
        .whileFalse(new SetIntakeSpeed(m_intake, 0));

        // m_operatorController.start()
        // .whileTrue(new SetArmAngle(m_arm, desiredEncoderValue.kSpeakerArmAngle));

        // m_operatorController.back()
        // .whileTrue(new SetArmAngle(m_arm, desiredEncoderValue.kIntakeArmAngle));

        // m_operatorController.start().onTrue(m_arm.setArmGoalCommand(Units.degreesToRadians(30))); // --> change the radian 
        // m_operatorController.back().onTrue(m_arm.setArmGoalCommand(Units.degreesToRadians(30)));



        //left joystick charge up, press a button to move note in --> transfered to joystick controls 
        //  m_operatorController.x().whileTrue(m_arm.armClockWise());
        //  m_operatorController.y().whileTrue(m_arm.armCounterClockWise());
        
    // driver controls 
    /*
     * Right Bumper: climb down (right)
     * Left Bumper: climb down (left)
     * Right Trigger: climb down (right)
     * Left Trigger: climb down (left)
     */
         m_driverController.rightBumper().whileTrue(m_climber.climbDownRight());
         m_driverController.leftBumper().whileTrue(m_climber.climbDownLeft());
         m_driverController.rightTrigger().whileTrue(m_climber.climbUpRight());
         m_driverController.leftTrigger().whileTrue(m_climber.climbUpLeft());

         m_operatorController.leftTrigger().whileTrue(m_arm.armSpeakerAngle());
        
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // create test
    m_robotDrive.setBrake();
    //return new PathPlannerAuto("Example Auto");
    return autoChooser.getSelected();
    
    // Create config for trajectory
    /* 
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    */
  } 


  public void registerNamedCommands(){
    NamedCommands.registerCommand("Shooter On", new SetShooterSpeed(m_shooter, Constants.IntakeShooter.kTopShootSpeed, Constants.IntakeShooter.kBottomShootSpeed));
    NamedCommands.registerCommand("Stop Shooter", new SetShooterSpeed(m_shooter, 0, 0));
    NamedCommands.registerCommand("Intake On", new SetIntakeSpeed(m_intake, Constants.IntakeShooter.kIntakeSpeed));
    NamedCommands.registerCommand("Stop Intake", new SetIntakeSpeed(m_intake, 0));
    NamedCommands.registerCommand("Arm Speaker", new SetArmAngle(m_arm, Constants.desiredEncoderValue.kSpeakerArmAngle));
    NamedCommands.registerCommand("Arm Intake", new SetArmAngle(m_arm, Constants.desiredEncoderValue.kIntakeArmAngle));
    NamedCommands.registerCommand("Sensor Detect Stop", new DetectNoteHigh(m_intake));
    NamedCommands.registerCommand("Sensor Detect Stop", new DetectNoteLow(m_intake));

  }
}