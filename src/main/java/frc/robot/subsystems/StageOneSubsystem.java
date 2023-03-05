package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StageOneSubsystem extends SubsystemBase { 
    private WPI_TalonFX armExtendMotor;
    public StageOneSubsystem () {
        //TODO: fix all of these they're weird. this one is actually incorrect.
        armExtendMotor = new WPI_TalonFX(13);
    }
    public void setarmExtendMotor(double value){
    armExtendMotor.set(ControlMode.PercentOutput, value);
    }
}