// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Stage1Command;
import frc.robot.commands.AutoDistance;
import frc.robot.commands.ClawArmPivotCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.PivotClawCommand;
import frc.robot.commands.RotateArmCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.Stage2Command;
import frc.robot.subsystems.Stage1Subsystem;
import frc.robot.subsystems.ClawArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotClawSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.Stage2Subsystem;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, driverController);
  private final AutoDistance autoDistance = new AutoDistance(driveSubsystem);

  //private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  //private final ClawCommand outClawCommand = new ClawCommand(clawSubsystem, .7);
  //private final ClawCommand inClawCommand = new ClawCommand(clawSubsystem, -.7);

  //private final PivotClawSubsystem pivotClawSubsystem = new PivotClawSubsystem();
  //private final PivotClawCommand pivotDownCommand = new PivotClawCommand(pivotClawSubsystem, .1);
  //private final PivotClawCommand pivotUpCommand = new PivotClawCommand(pivotClawSubsystem, -.1);

  //private final ArmExtendSubsystem armExtendSubsystem = new ArmExtendSubsystem();
  //private final ArmExtendCommand armExtendCommand = new ArmExtendCommand(armExtendSubsystem, 0);

  //private final UpperArmSubsystem upperArmSubsystem = new UpperArmSubsystem();
  //private final UpperArmCommand upperArmOutCommand = new UpperArmCommand(upperArmSubsystem, .3);
  //private final UpperArmCommand upperArmDownCommand = new UpperArmCommand(upperArmSubsystem, -.3);

  //private final RotateArmSubsystem rotateArmSubsystem = new RotateArmSubsystem();
  //private final RotateArmCommand armForwardCommand = new RotateArmCommand(rotateArmSubsystem, .3);
  //private final RotateArmCommand armBackwardCommand = new RotateArmCommand(rotateArmSubsystem, -.3);

  //private final ClawArmPivotSubsystem clawArmPivotSubsystem = new ClawArmPivotSubsystem();
  //private final ClawArmPivotCommand pivotClawArmForward = new ClawArmPivotCommand(clawArmPivotSubsystem, .1);
  //private final ClawArmPivotCommand pivotClawArmBack = new ClawArmPivotCommand(clawArmPivotSubsystem, -.1);

  SendableChooser<String> autoChooser = new SendableChooser<>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Add new path to this chooser to select them from shuffleboard.
    autoChooser.setDefaultOption("Outtake 1", "Outtake 1");
    autoChooser.addOption("Top 2 Object Scale", "T2S");
    autoChooser.addOption("Top 2 Object", "T2");
    autoChooser.addOption("Top 3 Object", "T3");
    autoChooser.addOption("Bottom 2 Object Scale", "B2S");
    autoChooser.addOption("Bottom 2 Object", "B2");
    autoChooser.addOption("Test Path 1", "Test Path 1");
    autoChooser.addOption("Test Path 2", "Test Path 2");
    autoChooser.addOption("180", "Test Path 3");
    autoChooser.addOption("Just 180", "Just 180");
    autoChooser.addOption("To Cone", "To Cone");
    autoChooser.addOption("Long Path", "The Long Path");
    SmartDashboard.putData("Path Chooser", autoChooser);
    
    //green larson
    //green larson
    // Configure the trigger bindings
    configureBindings();
  }

  //green larson
  private void configureBindings() {
    //driverController.x().whileTrue(pivotClawArmForward);
    //driverController.b().whileTrue(pivotClawArmBack);
    //driverController.
  }

  public Command getTeleopCommand() {
    return teleopCommand;
  }

  public Command distanceTest() {
    return autoDistance;
  }

  public Command getAutonomousCommand() {
    
    if(!autoChooser.getSelected().equals("Outtake 1")) {
      PathPlannerTrajectory path = PathPlanner.loadPath(autoChooser.getSelected(), new PathConstraints(1.5,2));

      HashMap<String, Command> eventMap = new HashMap<>();
      //eventMap.put("Stop", stopCommand);
      //eventMap.put("Intake", IntakeCommand());    <-- Uncomment when these commands exist
      //eventMap.put("Outtake", OuttakeCommand());  <--
  
      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        driveSubsystem::getPose, // Pose2d supplier
        driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        driveSubsystem.kinematics, // SwerveDriveKinematics

        //originally 6, 0, 0
        //tried 4.5, 0, .007
        //green larson
        //new PIDConstants(5.7, .003, 0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //change this next line only
        //p up, i down, d up i guess
        new PIDConstants(6.5, 0.000, .07),
        //d was originally .004 for turn, check that?
        // IGNORE BELOW
        new PIDConstants(2.55, .01374, .008), // PID constants to correct for rotation error (used to create the rotation controller)
        driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
      );
      return autoBuilder.fullAuto(path); 
    }

    // Replace with outtake command.
    return null;

    /*
    // Uncomment if autobuilder does not work properly.
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(autoChooser.getSelected(),  new PathConstraints(1.5,2));
 
    HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("Intake", IntakeCommand());    <-- Uncomment when these commands exist
    //eventMap.put("Outtake", OuttakeCommand());  <--
 
    FollowPathWithEvents command = new FollowPathWithEvents(
      driveSubsystem.followTrajectoryCommand(trajectory, true),
      trajectory.getMarkers(),
      eventMap
    );

    return command;
    */
  }
}
