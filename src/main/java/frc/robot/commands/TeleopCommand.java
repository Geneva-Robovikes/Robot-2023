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
  private final double maxSpeedTheta = Math.PI / 2;

  public TeleopCommand(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double x1 = controller.getLeftX();
    double y1 = controller.getLeftY();
    double x2 = controller.getRightX();

    x1 = MathUtil.applyDeadband(x1, OperatorConstants.controllerDeadzone);
    y1 = MathUtil.applyDeadband(x1, OperatorConstants.controllerDeadzone);
    x2 = MathUtil.applyDeadband(x1, OperatorConstants.controllerDeadzone);

    double vX = x1 * maxSpeedX;
    double vY = y1 * maxSpeedY;
    double vTheta = x2 * maxSpeedTheta;

    //how to move robot
    driveSubsystem.setModuleStatesFromSpeeds(vX, vY, vTheta);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
