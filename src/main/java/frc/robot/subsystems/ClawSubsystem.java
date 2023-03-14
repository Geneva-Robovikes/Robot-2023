package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {
    private final WPI_TalonFX pivotMotor;
    private final CANSparkMax clawMotor;
    private final DigitalInput pivotLimitSwitchTop;
    private final DigitalInput pivotLimitSwitchBottom;

    public ClawSubsystem(){
        pivotMotor = new WPI_TalonFX(9);
        clawMotor = new CANSparkMax(8, MotorType.kBrushless);
        
        pivotLimitSwitchTop = new DigitalInput(2);
        pivotLimitSwitchBottom = new DigitalInput(3);

        clawMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setClawMotor(double value){
        clawMotor.set(value);
    }

    public void setPivotMotor(double speed){
        pivotMotor.set(speed);
    }

    public boolean getPivotTopState() {
        return !pivotLimitSwitchTop.get();
    }

    public boolean getPivotBottomState() {
        return pivotLimitSwitchBottom.get();
    }

    public double getPivotDistance() {
        return pivotMotor.getSelectedSensorPosition();
    }
    
    public void resetPivotEncoder() {
        pivotMotor.setSelectedSensorPosition(0);
    }

    public double getClawAngle() {
        return pivotMotor.getSelectedSensorPosition() / Constants.falconEncoderResolution / Constants.clawAngleGearRatio * (2 * Math.PI);
    }
}
