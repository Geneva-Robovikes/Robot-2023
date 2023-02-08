// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController controller;

  //TODO: Test and get good values. All in m/s
  private final double maxSpeedX = 1;
  private final double maxSpeedY = 1;
  private final double maxSpeedTheta = Math.PI;
  boolean isFieldCentric = true;

  public TeleopCommand(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double x1 = -Math.signum(controller.getLeftX()) * Math.pow(controller.getLeftX(), 2);
    double y1 = -Math.signum(controller.getLeftY()) * Math.pow(controller.getLeftY(), 2);
    double x2 = -Math.signum(controller.getRightX()) * Math.pow(controller.getRightX(), 2);
    double rightTrigger = controller.getRightTriggerAxis();

    if(rightTrigger > 0.5) {
      isFieldCentric = false;
    } else {
      isFieldCentric = true;
    }

    if(controller.y().getAsBoolean()) {
      driveSubsystem.resetGyro();
    }

    x1 = MathUtil.applyDeadband(x1, OperatorConstants.controllerDeadzone);
    y1 = MathUtil.applyDeadband(y1, OperatorConstants.controllerDeadzone);
    x2 = MathUtil.applyDeadband(x2, OperatorConstants.controllerDeadzone);

    double vX = y1 * maxSpeedX;
    double vY = x1 * maxSpeedY;
    double vTheta = x2 * maxSpeedTheta;

    driveSubsystem.setModuleStatesFromSpeeds(vX, vY, vTheta, isFieldCentric);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0, isFieldCentric);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
