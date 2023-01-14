// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorDownCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private double maxHeight = 307200;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorDownCommand(ElevatorSubsystem subsystem) {
    elevatorSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setelevatorMotor(-0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(elevatorSubsystem.getelevatorEncoder() > maxHeight) {return true;}
    else {return false;}
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setelevatorMotor(0);
  }
}