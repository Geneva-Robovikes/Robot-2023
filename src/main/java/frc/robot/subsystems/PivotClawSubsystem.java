package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PivotClawSubsystem extends SubsystemBase {
    private WPI_TalonFX pivotMotor;

    DigitalInput pivotClawSubsystemLimitSwitch1;
    DigitalInput pivotClawsubsystemLimitSwitch2;

    public PivotClawSubsystem(){
        pivotClawSubsystemLimitSwitch1 = new DigitalInput(2);
        pivotClawsubsystemLimitSwitch2 = new DigitalInput(3);
        pivotMotor = new WPI_TalonFX(9);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
    }
    public void setPivotMotor(double speed){
        pivotMotor.set(ControlMode.PercentOutput, speed);
    }
    public boolean getSwitchState() {
        return (pivotClawSubsystemLimitSwitch1.get()||pivotClawsubsystemLimitSwitch2.get());
    }
}
