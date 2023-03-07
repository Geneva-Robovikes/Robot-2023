package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawArmPivotSubsystem extends SubsystemBase {
    private WPI_TalonFX upperArmClawPivotMotorAAAA;
    DigitalInput clawArmLimitSwitch1;
    DigitalInput clawArmLimitSwitch2;
    
    public ClawArmPivotSubsystem(){
        //TODO: set to real input
        //green larson
        clawArmLimitSwitch1 = new DigitalInput(0);
        clawArmLimitSwitch2 = new DigitalInput(1);
        upperArmClawPivotMotorAAAA = new WPI_TalonFX(12);
        upperArmClawPivotMotorAAAA.setNeutralMode(NeutralMode.Brake);
    }

    public void setArmMotor(double speed) {
        upperArmClawPivotMotorAAAA.set(speed);
    }

    public double getArmPosition() {
        return upperArmClawPivotMotorAAAA.getSelectedSensorPosition();
    }

    public boolean getSwitchState() {
        return (clawArmLimitSwitch1.get()||clawArmLimitSwitch2.get());
        
    }

}
