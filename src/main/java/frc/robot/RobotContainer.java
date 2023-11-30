// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StageOneDistanceCommand;
import frc.robot.commands.AutoBackUpCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoClawArmPivotCommand;
import frc.robot.commands.AutoClawCommand;
import frc.robot.commands.AutoPivotClawCommand;
import frc.robot.commands.ClawArmPivotCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.FullArmCommand;
import frc.robot.commands.JoystickControlCommand;
import frc.robot.commands.PivotClawCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.StageTwoDistanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController controlController = new CommandXboxController(OperatorConstants.kControlControllerPort);

  // The robot's subsystems and commands are defined here...
  
  /* ~~~ Subsystems ~~~ */
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();

  /* ~~~~ Commands ~~~~ */
  private final ClawCommand clawInCommand = new ClawCommand(clawSubsystem, -.5, 35, .25);
  private final ClawCommand clawOutCommand = new ClawCommand(clawSubsystem, 1 , 50, .25);

  SendableChooser<String> autoChooser = new SendableChooser<>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    // Add new auto paths to this chooser to select them from shuffleboard.
    autoChooser.setDefaultOption("Outtake 1", "Outtake 1");
    autoChooser.addOption("Outtake 1 Balance", "Outtake 1 Balance");
    autoChooser.addOption("Top 1.5 Object Scale", "T1.5S");
    autoChooser.addOption("Top 2 Object Scale", "T2S");
    autoChooser.addOption("Top 2 Object", "T2");
    autoChooser.addOption("Top 3 Object", "T3");
    autoChooser.addOption("Bottom 1.5 Object Scale", "B1.5S");
    autoChooser.addOption("Bottom 2 Object Scale", "B2S");
    autoChooser.addOption("Bottom 2 Object", "B2");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Adds all limit switches to shuffleboard
   */
  public void checkLimitSwitch() {
    SmartDashboard.putBoolean("1-down", armSubsystem.getLowerExtensionBottomState());
    SmartDashboard.putBoolean("1-up", armSubsystem.getLowerExtensionTopState());
    SmartDashboard.putBoolean("2-down", armSubsystem.getUpperExensionBottomState());
    SmartDashboard.putBoolean("2-up", armSubsystem.getUpperExensionTopState());
    SmartDashboard.putBoolean("Arm Down", armSubsystem.getArmPivotBottomState());
    SmartDashboard.putBoolean("Arm Up", armSubsystem.getArmPivotTopState());
    SmartDashboard.putBoolean("Claw Down", clawSubsystem.getPivotBottomState());
    SmartDashboard.putBoolean("Claw Up", clawSubsystem.getPivotTopState());
    SmartDashboard.putNumber("x", controlController.getLeftX());
    SmartDashboard.putNumber("y", controlController.getLeftY());
    SmartDashboard.putNumber("gyro", driveSubsystem.getGyroAngleY());
  }

  /**
   * Puts encoder values on shuffleboard
   */
  public void encoderTest() {
    SmartDashboard.putNumber("claw pivot distance", clawSubsystem.getPivotDistance());
    SmartDashboard.putNumber("arm pivot distance", armSubsystem.getArmPivotPosition());
    SmartDashboard.putNumber("First Stage distance", armSubsystem.getLowerExtensionDistance());
    SmartDashboard.putNumber("Second Stage distance", armSubsystem.getUpperExensionDistance());
  }

  /** Setup for controller buttons */
  private void configureBindings() {
    controlController.rightBumper().whileTrue(new FullArmCommand(armSubsystem, 0.75));
    controlController.leftBumper().whileTrue(new FullArmCommand(armSubsystem, -0.75));
    controlController.rightTrigger().whileTrue(clawOutCommand);
    controlController.leftTrigger().whileTrue(clawInCommand.andThen(new ControllerRumbleCommand(controlController, 0.5, 0.5)));
  }

  /** 
   * Returns the command to run in teleop control. Usually sends the command to Robot.java teleop init.
   * @return the command to run in Teleop.
   */
  public Command getTeleopCommand() {
    return new ParallelCommandGroup(
      new TeleopCommand(driveSubsystem, driverController),
      new JoystickControlCommand(controlController, armSubsystem, clawSubsystem, 0.6, 0.6)
    );
  }

  /** 
   * Returns the command to run in autonomous control. Usually sends the command to Robot.java autonomous init.
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    driveSubsystem.resetGyro();

    //Small scary sequence of commands for putting one thing on the top level
    if(autoChooser.getSelected().equals("Outtake 1")) {
      return new ParallelCommandGroup(
        new FullArmCommand(armSubsystem, 0.5),
        new AutoClawArmPivotCommand(armSubsystem, -0.4, 115000)
        ).andThen(new AutoClawCommand(clawSubsystem, 1, 0.5)
        ).andThen(new ParallelCommandGroup(
        new FullArmCommand(armSubsystem, -0.5),
        new ClawArmPivotCommand(armSubsystem, 0.6, true),
        new PivotClawCommand(clawSubsystem, 0.27, false)
        ).andThen(new AutoBackUpCommand(driveSubsystem, -0.6, 4.25, true)));
    }

    // Big scary sequence of commands for putting one thing on the top level and balancing
    else if(autoChooser.getSelected().equals("Outtake 1 Balance")) {
      driveSubsystem.resetGyro();
      return new AutoBalance(driveSubsystem, -0.225, -0.45, 5, 1.75);
      /*return new ParallelCommandGroup(
        new FullArmCommand(armSubsystem, 0.75),
        new AutoClawArmPivotCommand(armSubsystem, -0.4, 115000)
        ).andThen(new AutoClawCommand(clawSubsystem, 1, 0.5)
        ).andThen(new ParallelCommandGroup(
        new FullArmCommand(armSubsystem, -0.75),
        new ClawArmPivotCommand(armSubsystem, 0.6, true),
        new PivotClawCommand(clawSubsystem, 0.27, false)
        ).andThen(new WaitCommand(0.5)
        ).andThen(new AutoBalance(driveSubsystem, -0.225, -0.45, 5, 1.75))
      );*/
    }

    // Makes sure the selected auto is a valid path
    if(!autoChooser.getSelected().equals("Outtake 1") && !autoChooser.getSelected().equals("Outtake 1 Balance")) { 

      // Creates the path from the path file that was selected from shuffleboard
      ArrayList<PathPlannerTrajectory> path = new ArrayList<>(PathPlanner.loadPathGroup(autoChooser.getSelected(), 1, 0.5));
      
      // Creates the hashmap to put all the commands onto.
      // Anything used in path planner must be declred here to function.
      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("Cone Outtake", new AutoClawCommand(clawSubsystem, 1, 0.5));
      eventMap.put("Cube Outtake", new AutoClawCommand(clawSubsystem, 1, 0.75));
      eventMap.put("Intake", new ClawCommand(clawSubsystem, -0.5, 25, 0.25));
      eventMap.put("Lower Arm", new ParallelCommandGroup(
        new StageTwoDistanceCommand(armSubsystem, .25, 44000),
        new StageOneDistanceCommand(armSubsystem, .25, 33000),
        new ClawArmPivotCommand(armSubsystem, -0.6, false),
        new PivotClawCommand(clawSubsystem, -0.27, true)
      ));
      eventMap.put("Raise Arm", new ParallelCommandGroup(
        new FullArmCommand(armSubsystem, -0.5),
        new ClawArmPivotCommand(armSubsystem, 0.6, true),
        new PivotClawCommand(clawSubsystem, 0.27, false)
      ));
      
      eventMap.put("Place Cube",  new ParallelCommandGroup(
        new AutoClawArmPivotCommand(armSubsystem, -0.4, 115000),
        new AutoPivotClawCommand(clawSubsystem, -0.35, 97500)
      ));
      eventMap.put("Place Cone", new ParallelCommandGroup(
        new FullArmCommand(armSubsystem, 0.5),
        new AutoClawArmPivotCommand(armSubsystem, -0.4, 115000)
      ));



  
      // The auto builder is used to create a full autonomus routine.
      // This will combine paths and commands into a full sequence.
      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        driveSubsystem::getPose, // Pose2d supplier
        driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        driveSubsystem.kinematics, // SwerveDriveKinematics
        new PIDConstants(3.1679, 0, 0),
        new PIDConstants(4.1807, 0, 0.23405),
        //new PIDConstants(6.5, 0.000, .07), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //new PIDConstants(2.55, .01374, .008), // PID constants to correct for rotation error (used to create the rotation controller)
        driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
      );
      
      return autoBuilder.fullAuto(path);
    }

    return null;
  }
}