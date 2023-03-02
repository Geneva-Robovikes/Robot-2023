package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax clawMotor ; 

    public ClawSubsystem(){
        clawMotor = new CANSparkMax(10, MotorType.kBrushless);
    }
    public void setClawMotor(double value){
        clawMotor.set(value);
    }
}
