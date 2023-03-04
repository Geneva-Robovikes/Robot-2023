package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawArmPivotSubsystem extends SubsystemBase {
    private WPI_TalonFX upperArmClawPivotMotorAAAA;
    
    public ClawArmPivotSubsystem(){
        upperArmClawPivotMotorAAAA = new WPI_TalonFX(12);
    }

    public void turnArmMotor(double speed) {
        upperArmClawPivotMotorAAAA.set(speed);
    }
}
