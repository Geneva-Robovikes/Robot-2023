// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MoveToTargetCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  CameraSubsystem[] cameras = { new CameraSubsystem("Front Cam",
                                  new Transform3d(
                                    new Pose3d(.0, .0, .0, new Rotation3d()),
                                    //TODO: Set to real pose
                                    new Pose3d(.0, .0, .0, new Rotation3d())
                                  ))/* ,
                                new CameraSubsystem("Back Cam",
                                  new Transform3d(
                                    new Pose3d(.0, .0, .0, new Rotation3d()),
                                    //TODO: Set to real pose
                                    new Pose3d(.0, .0, .0, new Rotation3d(new Quaternion(1, 0, 0, 0)))
))};*/ };

  CameraSubsystem camera1 = cameras[0];
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(new int[] {0, 1, 2, 3, 4, 5, 6, 7}, cameras);
  private final TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, driverController);
  private final MoveToTargetCommand moveToTargetCommand = new MoveToTargetCommand(camera1, driveSubsystem);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return moveToTargetCommand;
    //return null;
  }

  public Command getTeleopCommand() {
    return teleopCommand;
  }
}
