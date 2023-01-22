// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(new int[] {0, 1, 2, 3, 4, 5, 6, 7});

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand(String pathName) {
    PathPlannerTrajectory trajectory;
    if(!pathName.equals("One Object")) {
      //TODO: Add path selection to shuffleboard
      trajectory = PathPlanner.loadPath(pathName,  new PathConstraints(1,1));
      
      HashMap<String, Command> eventMap = new HashMap<>();
      //eventMap.put("Intake", IntakeCommand());    <-- Uncomment when these commands exist
      //eventMap.put("Outtake", OuttakeCommand());  <--
      //eventMap.put("Balance", BalanceCommand());  <--
      
      FollowPathWithEvents command = new FollowPathWithEvents(
        driveSubsystem.followTrajectoryCommand(trajectory, true),
        trajectory.getMarkers(),
        eventMap
      );

      return command;
    }

    //TODO: set to outtake command, maybe back onto the scale too
    return null;
  }
}
