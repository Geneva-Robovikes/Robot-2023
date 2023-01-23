// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorUpCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  //private final DriveSubsystem driveSubsystem;

  //TODO: Add elevator length to contructor-done
  double length;
  

  public ElevatorUpCommand(ElevatorSubsystem subsystem, double length) {
    elevatorSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.length = length;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setelevatorLength(length);
  }

  // Called every time the scheduler runs while the command is scheduled.
  //TODO: set motor using PID in execute done?
  @Override
  public void execute() {
    elevatorSubsystem.setelevatorLength(length);
  
  }

  // Returns true when the command should end.
  //TODO: Add finish condition done?
  
  @Override
  public boolean isFinished() {
    if(elevatorSubsystem.atSetPoint()) {return true;}
    else {return false;}
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setelevatorMotor(0);
  }
}