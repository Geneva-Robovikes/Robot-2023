// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorUpCommand extends CommandBase {
  private final ElevatorSubsystem ElevatorSubsystem;
  //private final DriveSubsystem driveSubsystem;

  //TODO: Add elevator length to contructor
  //done??
  public ElevatorUpCommand(ElevatorSubsystem subsystem, double length) {
    ElevatorSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSubsystem.setelevatorMotor(.35, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  //TODO: set motor using PID in execute
  @Override
  public void execute() {
    ElevatorSubsystem.setelevatorMotor(.35, 0);
    PIDController pid = new PIDController(kP, kI, kD);
  }

  // Returns true when the command should end.
  //TODO: Add finish condition
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    ElevatorSubsystem.setelevatorMotor(0, 0);
  }
}