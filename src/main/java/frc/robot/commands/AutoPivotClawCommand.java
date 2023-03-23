package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotClawSubsystem;

public class AutoPivotClawCommand extends CommandBase {
    private PivotClawSubsystem pivotClawSubsystem;
    private double speed; 
    private double distance;

    private ClawSubsystem clawSubsystem;

    public AutoPivotClawCommand (PivotClawSubsystem subsystem, double speed, double distance){
        pivotClawSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    public AutoPivotClawCommand (ClawSubsystem subsystem, double speed, double distance) {
        this.clawSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        //pivotClawSubsystem.setControl(false);
        //pivotClawSubsystem.resetEncoder();
        //pivotClawSubsystem.setPivotMotor(speed);
        clawSubsystem.resetPivotEncoder();
        clawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public void execute() {
        //pivotClawSubsystem.setPivotMotor(speed);
        clawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public boolean isFinished() {
        //return Math.abs(pivotClawSubsystem.getDistance()) > distance;
        return Math.abs(clawSubsystem.getPivotDistance()) > distance;

        //untested.
        /*if(speed<0) {
            return clawSubsystem.getPivotBottomState() || Math.abs(clawSubsystem.getPivotDistance()) > distance;
        } else {
            return clawSubsystem.getPivotBottomState() || Math.abs(clawSubsystem.getPivotDistance()) > distance;
        }*/
    }
    
    @Override
    public void end(boolean interrupted) {
        //pivotClawSubsystem.setPivotMotor(0);
        //pivotClawSubsystem.setControl(true);
        clawSubsystem.setPivotMotor(0);
    }
}
