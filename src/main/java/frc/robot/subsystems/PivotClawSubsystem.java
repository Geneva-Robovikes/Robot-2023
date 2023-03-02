package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotClawSubsystem extends SubsystemBase {
    private WPI_TalonFX pivotMotor;

    public PivotClawSubsystem(){
        pivotMotor = new WPI_TalonFX(8);
    }
    public void setPivotMotor(double value){
        pivotMotor.set(ControlMode.PercentOutput, value);
    }
}
