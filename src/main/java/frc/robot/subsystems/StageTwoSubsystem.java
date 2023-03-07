package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StageTwoSubsystem extends SubsystemBase{
    WPI_TalonFX upperArmMotor;
    DigitalInput stageTwoLimitSwitchBottom;
    DigitalInput stageTwoLimitSwitchTop;

    public StageTwoSubsystem() {
        stageTwoLimitSwitchBottom = new DigitalInput(6);
        stageTwoLimitSwitchTop = new DigitalInput(7);
        upperArmMotor = new WPI_TalonFX(10);
        upperArmMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setUpperMotor(double speed) {
        upperArmMotor.set(speed);
    }
    public boolean getSwitchState() {
        return (stageTwoLimitSwitchBottom.get()||stageTwoLimitSwitchTop.get());
    }
}

//green larson//green larson
//green larson
