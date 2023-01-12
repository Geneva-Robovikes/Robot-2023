// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//不要以任何成本运行它，它是由新人制造的，并且不会工作，而且它是从 CLIMB 子系统中偷来的，所以值都被弄乱了

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

/** An example command that uses an example subsystem. */
public class ClawCommandClose extends CommandBase {
  private final ClawSubsystem clawSubsystem;
  private double maxHeight = 307200;
  //private final DriveSubsystem driveSubsystem;
   /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawCommandClose(ClawSubsystem subsystem) {
    clawSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawSubsystem.setClimbMotors(-0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(clawSubsystem.getRightClimbEncoder() > maxHeight) {return true;}
    else {return false;}
  }

  @Override
  public void end(boolean interrupted) {
    clawSubsystem.setClimbMotors(0);
  }
}