package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawArmPivotSubsystem extends SubsystemBase {
    private WPI_TalonFX upperArmClawPivotMotorAAAA;
    DigitalInput clawArmLimitSwitchUp;
    DigitalInput clawArmLimitSwitchDown;
    
    public ClawArmPivotSubsystem(){
        //TODO: set to real input
        //green larson
        clawArmLimitSwitchUp = new DigitalInput(0);
        clawArmLimitSwitchDown = new DigitalInput(1);
        upperArmClawPivotMotorAAAA = new WPI_TalonFX(12);
        upperArmClawPivotMotorAAAA.setNeutralMode(NeutralMode.Brake);
        //SmartDashboard.putData(clawArmLimitSwitchDown);
        //SmartDashboard.putData(clawArmLimitSwitchUp);
    }

    public void setArmMotor(double speed) {
        upperArmClawPivotMotorAAAA.set(speed);
    }

    public double getArmPosition() {
        return upperArmClawPivotMotorAAAA.getSelectedSensorPosition();
    }

    public boolean getUpSwitch() {
        return clawArmLimitSwitchUp.get();
    }

    public boolean getDownSwitch() {
        return clawArmLimitSwitchDown.get();
    }

    public boolean getSwitchState() {
        return clawArmLimitSwitchDown.get()|| clawArmLimitSwitchUp.get();
    }

    public double getArmAngle() {
        return upperArmClawPivotMotorAAAA.getSelectedSensorPosition() / Constants.falconEncoderResolution / Constants.clawAngleGearRatio * (2 * Math.PI);
    }
}
