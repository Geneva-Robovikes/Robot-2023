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
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
  private final ClawCommand clawInCommand = new ClawCommand(clawSubsystem, -.5, 25, .25);
  private final ClawCommand clawOutCommand = new ClawCommand(clawSubsystem, 1 ,25, .25);
  private final FullArmCommand fullArmCommand = new FullArmCommand(armSubsystem, 0);

  //private final FullArmCommand fullArmUpCommand = new FullArmCommand(stageOneSubsystem, stageTwoSubsystem, -.25, .25);
  //private final FullArmCommand fullArmDownCommand = new FullArmCommand(stageOneSubsystem, stageTwoSubsystem, .25, -.25);

  SendableChooser<String> autoChooser = new SendableChooser<>(); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    // Add new path to this chooser to select them from shuffleboard.
    autoChooser.setDefaultOption("Third Level Cube", "Third Level Cube");
    autoChooser.addOption("Third Level Cone", "Third Level Cone");
    autoChooser.addOption("Second Level Cone", "Second Level Cone");
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
      new StageTwoDistanceCommand(armSubsystem, .25, 35000),
      new StageOneDistanceCommand(armSubsystem, .25, 32000),
      new ClawArmPivotCommand(armSubsystem, -0.4, false),
      new PivotClawCommand(clawSubsystem, -0.2, true)
    ));
    controlController.rightBumper().whileTrue(new FullArmCommand(armSubsystem, 0.5));
    controlController.leftBumper().whileTrue(new FullArmCommand(armSubsystem, -0.5));
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

    Command startingPart;
    if(autoChooser.getSelected().equals("Third Level Cube")) {
      //startingPart = new ParallelCommandGroup(new AutoClawArmPivotCommand(clawArmPivotSubsystem, -0.32, 198864), new AutoPivotClawCommand(pivotClawSubsystem, -0.1, 40000)).andThen(new AutoTimedClawCommand(clawSubsystem, 0.5, 0.25));
      startingPart = new ParallelCommandGroup(new AutoClawArmPivotCommand(armSubsystem, 0, 0), new AutoPivotClawCommand(clawSubsystem, 0, 0)).andThen(new AutoTimedClawCommand(clawSubsystem, .5, .25));
    } else if (autoChooser.getSelected().equals("Third Level Cone")) {
      //startingPart = new ParallelCommandGroup(new ParallelRaceGroup(new ParallelCommandGroup(new StageTwoCommand(stageTwoSubsystem, 0.3, true), new StageOneCommand(stageOneSubsystem, -0.3, true) ,new AutoClawArmPivotCommand(clawArmPivotSubsystem, -0.32, 198864), new AutoPivotClawCommand(pivotClawSubsystem, -0.1, 40000)), new AutoClawCommand(clawSubsystem, -0.5, 1, 0.15)).andThen(new AutoClawArmPivotCommand(clawArmPivotSubsystem, -0.2, 30000).andThen(new AutoTimedClawCommand(clawSubsystem, 0.5, 0.25))));
    } else {
      //startingPart = new ParallelRaceGroup(new ParallelCommandGroup(new AutoClawArmPivotCommand(clawArmPivotSubsystem, -0.32, 198864), new AutoPivotClawCommand(pivotClawSubsystem, -0.1, 40000)), new AutoClawCommand(clawSubsystem, -0.5, 1, 0.15)).andThen(new AutoClawArmPivotCommand(clawArmPivotSubsystem, -0.2, 30000).andThen(new AutoTimedClawCommand(clawSubsystem, 0.5, 0.25)));
    }

    //Command collapse = new ParallelCommandGroup(new ClawArmPivotCommand(clawArmPivotSubsystem, 0.32, true), new PivotClawCommand(pivotClawSubsystem, 0.1, false), new StageOneCommand(stageOneSubsystem, 0.3, false), new StageTwoCommand(stageTwoSubsystem, -0.3, false));

    if(!autoChooser.getSelected().equals("Outtake 1")) {
      PathPlannerTrajectory path = PathPlanner.loadPath("Straight Back", new PathConstraints(0.5,0.5));

      HashMap<String, Command> eventMap = new HashMap<>();
      //eventMap.put("Stop", stopCommand);            <-- Uncomment when these commands exist
      //eventMap.put("Intake", IntakeCommand());      <--
      //eventMap.put("Outtake", OuttakeCommand());    <--
      //eventMap.put("Lower Arm", LowerArmCommand()); <--
      //eventMap.put("Raise Arm", RaiseArmCommand()); <--
      //eventMap.put("Balance", BalanceCommand());    <--
  
      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        driveSubsystem::getPose, // Pose2d supplier
        driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        driveSubsystem.kinematics, // SwerveDriveKinematics
        new PIDConstants(6.5, 0.000, .07), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(2.55, .01374, .008), // PID constants to correct for rotation error (used to create the rotation controller)
        driveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
      );
      
      //return autoBuilder.fullAuto(path);
      //return startingPart.andThen(new ParallelCommandGroup(collapse, new AutoBackUpCommand(driveSubsystem, 0.6, 4.25, true))).andThen(new WaitCommand(0.5)).andThen(new AutoBackUpCommand(driveSubsystem, -0.6, 2, true).andThen(new AutoBalance(driveSubsystem, 0.225, 0.35, 5, 2.5))); 
      //return new AutoBackUpCommand(driveSubsystem, .4, 2, false).andThen(new AutoBalance(driveSubsystem, .225, .35, 5, 2.5));
    }

    // Replace with outtake one command.
    return null;
  }
}