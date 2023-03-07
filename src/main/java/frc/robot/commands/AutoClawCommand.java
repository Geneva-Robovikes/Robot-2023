package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawArmPivotSubsystem;
import frc.robot.subsystems.PivotClawSubsystem;

public class AutoClawCommand extends CommandBase {

    PivotClawSubsystem pivotClawSubsystem;
    ClawArmPivotSubsystem clawArmPivotSubsystem;
    double speedOne;
    double speedTwo;
    double position;
    public AutoClawCommand(PivotClawSubsystem pivotClawSubsystem, ClawArmPivotSubsystem clawArmPivotSubsystem, double speedOne, double speedTwo) {
        this.clawArmPivotSubsystem = clawArmPivotSubsystem;
        this.pivotClawSubsystem = pivotClawSubsystem;
        this.speedOne = speedOne;
        this.speedTwo = speedTwo;

    }

    @Override
    public void initialize() {
        clawArmPivotSubsystem.setArmMotor(speedOne);
        pivotClawSubsystem.setPivotMotor(speedTwo);
    }

    @Override
    public boolean isFinished() {
        return false;
        //green larson
        //green larson
    }

    @Override
    public void end(boolean interrupted) {
        pivotClawSubsystem.setPivotMotor(0);
        clawArmPivotSubsystem.setArmMotor(0);
    }
}
