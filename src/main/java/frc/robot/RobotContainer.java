// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PneumaticsCommand;
//import frc.robot.commands.TeleopCommand;
//import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  //private final DriveSubsystem driveSubsystem = new DriveSubsystem(new int[] {0, 1, 2, 3, 4, 5, 6, 7});
  //private final TeleopCommand teleopCommand = new TeleopCommand(driveSubsystem, driverController);

  public final PneumaticsCommand pneumaticsCommand = new PneumaticsCommand(pneumaticsSubsystem, false);



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
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    driverController.a().whileTrue(pneumaticsCommand);
    driverController.y().onTrue(pneumaticsSubsystem.setCubeModeCommand());
    //driverController.y().toggleOnTrue();

  }

  /*public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }*/

  /*public Command getTeleopCommand() {
    return teleopCommand;
  }*/
}
