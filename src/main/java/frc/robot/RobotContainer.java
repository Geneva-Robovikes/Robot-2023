// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.StageOneCommand;
import frc.robot.commands.StageOneDistanceCommand;
import frc.robot.commands.AutoBackUpCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoClawArmPivotCommand;
import frc.robot.commands.AutoClawCommand;
import frc.robot.commands.AutoDistance;
import frc.robot.commands.AutoPivotClawCommand;
import frc.robot.commands.AutoTimedClawCommand;
import frc.robot.commands.ClawArmPivotCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.FullArmCommand;
import frc.robot.commands.JoystickControlCommand;
import frc.robot.commands.PivotClawCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.StageTwoCommand;
import frc.robot.commands.StageTwoDistanceCommand;
import frc.robot.subsystems.StageOneSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotClawSubsystem;
import frc.robot.subsystems.StageTwoSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  //private final StageOneSubsystem stageOneSubsystem = new StageOneSubsystem();
  //private final StageTwoSubsystem stageTwoSubsystem = new StageTwoSubsystem();
  //private final ClawArmPivotSubsystem clawArmPivotSubsystem = new ClawArmPivotSubsystem();
  //private final PivotClawSubsystem pivotClawSubsystem = new PivotClawSubsystem();


  /* ~~~~ Commands ~~~~ */
  private final AutoDistance autoDistance = new AutoDistance(driveSubsystem);
  private final ClawCommand clawInCommand = new ClawCommand(clawSubsystem, -.5, 35, .25);
  private final ClawCommand clawOutCommand = new ClawCommand(clawSubsystem, 1 , 50, .25);

  //private final FullArmCommand fullArmUpCommand = new FullArmCommand(stageOneSubsystem, stageTwoSubsystem, -.25, .25);
  //private final FullArmCommand fullArmDownCommand = new FullArmCommand(stageOneSubsystem, stageTwoSubsystem, .25, -.25);

  SendableChooser<String> autoChooser = new SendableChooser<>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    // Add new path to this chooser to select them from shuffleboard.
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

  public void encoderTest() {
    SmartDashboard.putNumber("claw pivot distance", clawSubsystem.getPivotDistance());
    SmartDashboard.putNumber("arm pivot distance", armSubsystem.getArmPivotPosition());
    SmartDashboard.putNumber("First Stage distance", armSubsystem.getLowerExtensionDistance());
    SmartDashboard.putNumber("Second Stage distance", armSubsystem.getUpperExensionDistance());
  }

  /** Setup for controller buttons */
  private void configureBindings() {
    //controlController.y().whileTrue(new AutoBalance(driveSubsystem, 0.225, 0.35, 5, 2.5));

    controlController.a().whileTrue(new ParallelCommandGroup(
      new StageTwoDistanceCommand(armSubsystem, .25, 44000),
      new StageOneDistanceCommand(armSubsystem, .25, 33000),
      new ClawArmPivotCommand(armSubsystem, -0.4, false),
      new PivotClawCommand(clawSubsystem, -0.2, true)
    ));

    controlController.b().whileTrue(new ParallelCommandGroup(
      new StageTwoDistanceCommand(armSubsystem, .25, 37000),
      new StageOneDistanceCommand(armSubsystem, .25, 27000),
      new ClawArmPivotCommand(armSubsystem, -0.4, false),
      new PivotClawCommand(clawSubsystem, -0.2, true)
    ).andThen(new ParallelCommandGroup(
      new AutoClawArmPivotCommand(armSubsystem, 0.2, 30000),
      new AutoPivotClawCommand(clawSubsystem, .2, 15000))
    ));

    controlController.y().whileTrue(new ParallelCommandGroup(
      new FullArmCommand(armSubsystem, -0.5),
      new ClawArmPivotCommand(armSubsystem, 0.4, true),
      new PivotClawCommand(clawSubsystem, 0.2, false)
    ));

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

  public Command distanceTest() {
    return autoDistance;
  }

  /** 
   * Returns the command to run in autonomous control. Usually sends the command to Robot.java autonomous init.
   * @return the command to run in autonomous.
   */
  public Command getAutonomousCommand() {
    driveSubsystem.resetGyro();
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
  } else if(autoChooser.getSelected().equals("Outtake 1 Balance")) {
    driveSubsystem.resetGyro();
    return new ParallelCommandGroup(
      new FullArmCommand(armSubsystem, 0.75),
      new AutoClawArmPivotCommand(armSubsystem, -0.4, 115000)
      ).andThen(new AutoClawCommand(clawSubsystem, 1, 0.5)
      ).andThen(new ParallelCommandGroup(
      new FullArmCommand(armSubsystem, -0.75),
      new ClawArmPivotCommand(armSubsystem, 0.6, true),
      new PivotClawCommand(clawSubsystem, 0.27, false)
      ).andThen(new WaitCommand(0.5)
      ).andThen(new AutoBalance(driveSubsystem, -0.225, -0.45, 5, 1.75))
    );
  }
  /*if(autoChooser.getSelected().equals("Outtake 1 Cone Balance")) {
    driveSubsystem.resetGyro();
    return new ParallelCommandGroup(
      new FullArmCommand(armSubsystem, 0.75),
      new AutoClawArmPivotCommand(armSubsystem, -0.4, 115000)
      ).andThen(new AutoClawCommand(clawSubsystem, .75, 0.5)
      ).andThen(new ParallelCommandGroup(
      new FullArmCommand(armSubsystem, -0.75),
      new ClawArmPivotCommand(armSubsystem, 0.6, true),
      new PivotClawCommand(clawSubsystem, 0.27, false)
      /*).andThen(new AutoBackUpCommand(driveSubsystem, -0.6, 4, true)
      ).andThen(new WaitCommand(1) */
       //).andThen(new AutoBalance(driveSubsystem, -0.225, -0.45, 5, 1.75)));
  //}

    if(!autoChooser.getSelected().equals("Outtake 1") && !autoChooser.getSelected().equals("Outtake 1 Balance")) { 
      ArrayList<PathPlannerTrajectory> path = new ArrayList<>(PathPlanner.loadPathGroup(autoChooser.getSelected(), 1, 0.5));
      
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