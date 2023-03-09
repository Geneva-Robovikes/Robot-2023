package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class PivotClawSubsystem extends SubsystemBase {
    private WPI_TalonFX pivotMotor;

    DigitalInput pivotClawSubsystemLimitSwitch1;
    DigitalInput pivotClawSubsystemLimitSwitch2;

    public PivotClawSubsystem(){
        pivotClawSubsystemLimitSwitch1 = new DigitalInput(2);
        pivotClawSubsystemLimitSwitch2 = new DigitalInput(3);
        pivotMotor = new WPI_TalonFX(9);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setPivotMotor(double speed){
        pivotMotor.set(ControlMode.PercentOutput, speed);
    }

    public boolean getTopState() {
        return pivotClawSubsystemLimitSwitch1.get();
    }

    public boolean getBottomState() {
        return pivotClawSubsystemLimitSwitch2.get();
    }

    public boolean getSwitchState() {
        return (pivotClawSubsystemLimitSwitch1.get() || pivotClawSubsystemLimitSwitch2.get());
    }

    public double getClawAngle() {
        return pivotMotor.getSelectedSensorPosition() / Constants.falconEncoderResolution / Constants.clawAngleGearRatio * (2 * Math.PI);
    }
}
