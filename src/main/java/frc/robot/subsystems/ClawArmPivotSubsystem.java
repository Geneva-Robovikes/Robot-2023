package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawArmPivotSubsystem extends SubsystemBase {
    private WPI_TalonFX upperArmClawPivotMotorAAAA;
    DigitalInput clawArmLimitSwitch;
    
    public ClawArmPivotSubsystem(){
        //TODO: set to real input
        clawArmLimitSwitch = new DigitalInput(0);
        upperArmClawPivotMotorAAAA = new WPI_TalonFX(12);
        upperArmClawPivotMotorAAAA.setNeutralMode(NeutralMode.Brake);
    }

    public void turnArmMotor(double speed) {
        upperArmClawPivotMotorAAAA.set(speed);
    }

    public boolean getSwitchState() {
        return clawArmLimitSwitch.get();
    }
}
