package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClawArmPivotSubsystem;
import frc.robot.subsystems.PivotClawSubsystem;

public class JoystickControlCommand extends CommandBase {
    private PivotClawSubsystem pivotClawSubsystem;
    private ClawArmPivotSubsystem clawArmPivotSubsystem;
    private CommandXboxController controller;
    private double clawMaxSpeed; 
    private double armMaxSpeed;

    public JoystickControlCommand (CommandXboxController controller, PivotClawSubsystem subsystem, ClawArmPivotSubsystem subsystem2, double armMaxSpeed, double clawMaxSpeed){
        pivotClawSubsystem = subsystem;
        clawArmPivotSubsystem = subsystem2;
        this.clawMaxSpeed = clawMaxSpeed;
        this.armMaxSpeed = armMaxSpeed;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double leftY = controller.getRightY();
        double rightY = controller.getLeftY();

        if(((!pivotClawSubsystem.getTopState() && leftY < 0) || (!pivotClawSubsystem.getBottomState() && leftY > 0)) && pivotClawSubsystem.getControl())
            pivotClawSubsystem.setPivotMotor(leftY * clawMaxSpeed);
        else
            pivotClawSubsystem.setPivotMotor(0);
        
        if(((!clawArmPivotSubsystem.getUpSwitch() && rightY < 0) || (!clawArmPivotSubsystem.getDownSwitch() && rightY > 0)) && clawArmPivotSubsystem.getControl())
            clawArmPivotSubsystem.setArmMotor(-rightY * armMaxSpeed);
        else
            clawArmPivotSubsystem.setArmMotor(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        pivotClawSubsystem.setPivotMotor(0);
        clawArmPivotSubsystem.setArmMotor(0);
    }
}
