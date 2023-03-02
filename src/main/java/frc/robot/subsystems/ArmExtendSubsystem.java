package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmExtendSubsystem extends SubsystemBase { 
    private WPI_TalonFX armExtendMotor;
    public ArmExtendSubsystem () {
       armExtendMotor = new WPI_TalonFX(9);
    }
    public void setarmExtendMotor(double value){
    armExtendMotor.set(ControlMode.PercentOutput, value);
    }
}