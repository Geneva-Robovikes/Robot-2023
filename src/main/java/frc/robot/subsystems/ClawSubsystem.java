package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax clawMotor ; 

    public ClawSubsystem(){
        clawMotor = new CANSparkMax(8, MotorType.kBrushless);
        clawMotor.setIdleMode(IdleMode.kBrake);
    }
    public void setClawMotor(double value){
        clawMotor.set(value);
    }
}
