package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class JoystickControlCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final CommandXboxController controller;
    private final double clawMaxSpeed; 
    private final double armMaxSpeed;

    /**
     * Command that converts controller joystick inputs to arm, claw, and elevator motions.
     * @param controller the controller to get input from
     * @param armSubsystem the arm subsystem
     * @param clawSubsystem the claw subsystem
     * @param armMaxSpeed the max speed for the arm. Must be between 0.0 and 1.0
     * @param clawMaxSpeed the max speed for the claw roation. Must be between 0.0 and 1.0
     */
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

        if(((!clawSubsystem.getPivotTopState() && leftY < 0) || (!clawSubsystem.getPivotBottomState() && leftY > 0)) && Math.abs(leftY) > Constants.controllerDeadzone)
            clawSubsystem.setPivotMotor(leftY * clawMaxSpeed);
        else
            clawSubsystem.setPivotMotor(0);
        
        if(((!armSubsystem.getArmPivotTopState() && rightY < 0) || (!armSubsystem.getArmPivotBottomState() && rightY > 0)) && Math.abs(rightY) > Constants.controllerDeadzone)
            armSubsystem.setArmPivotMotor(-rightY * armMaxSpeed);
        else
            armSubsystem.setArmPivotMotor(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    //green larson
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setPivotMotor(0);
        armSubsystem.setArmPivotMotor(0);
    //Preheat oven to 350 degrees F (175 degrees C).
    //Grease a 9x9 inch baking 
    //In a medium bowl, cream 
    }
}
