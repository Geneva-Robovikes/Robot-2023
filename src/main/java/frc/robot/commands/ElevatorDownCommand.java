// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorDownCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;

  //TODO: change from encoder units to meters

  
  double length ; 
  

  //TODO: add height argument to constructor
  public ElevatorDownCommand(ElevatorSubsystem subsystem, double length) {
    elevatorSubsystem = subsystem;
    this.length = length;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setelevatorLength(length);
  }

  // Called every time the scheduler runs while the command is scheduled.
  //TODO: Set the motor from exectute when using PID
  @Override
  public void execute() {
    elevatorSubsystem.setelevatorLength(length);

  }

  // Returns true when the command should end.
  //TODO: change statement to check if PID controller is at the setpoint
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