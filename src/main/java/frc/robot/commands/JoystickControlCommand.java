package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class JoystickControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final CommandXboxController controller;
    private final double clawMaxSpeed; 
    private final double armMaxSpeed;

    public JoystickControlCommand (CommandXboxController controller, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem, double armMaxSpeed, double clawMaxSpeed){
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        this.clawMaxSpeed = clawMaxSpeed;
        this.armMaxSpeed = armMaxSpeed;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double leftY = controller.getRightY();
        double rightY = controller.getLeftY();

        if(((!clawSubsystem.getPivotTopState() && leftY < 0) || (!clawSubsystem.getPivotBottomState() && leftY > 0)))
            clawSubsystem.setPivotMotor(leftY * clawMaxSpeed);
        else
            clawSubsystem.setPivotMotor(0);
        
        if(((!armSubsystem.getArmPivotTopState() && rightY < 0) || (!armSubsystem.getArmPivotBottomState() && rightY > 0)))
            armSubsystem.setArmPivotMotor(-rightY * armMaxSpeed);
        else
            armSubsystem.setArmPivotMotor(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setPivotMotor(0);
        armSubsystem.setArmPivotMotor(0);
    }
}
