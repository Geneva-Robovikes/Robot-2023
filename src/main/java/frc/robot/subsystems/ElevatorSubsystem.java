package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private WPI_TalonFX elevatorMotor;
    private WPI_TalonFX angleMotor;
    private PIDController pid = new PIDController(0, 0, 0);

    public ElevatorSubsystem(int motorIndex, int angleIndex) {
        elevatorMotor = new WPI_TalonFX(motorIndex);
        angleMotor = new WPI_TalonFX(angleIndex);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

    }

    public double encoderToMeter(double encoder) {
        final double wheelDiameter = 0;
        return encoder / 2048 * wheelDiameter * Math.PI;
    }

    public void setelevatorLength(double length) {
        elevatorMotor.set(pid.calculate(encoderToMeter(elevatorMotor.getSelectedSensorPosition()), length));
    }

    public boolean atSetPoint() {
        return pid.atSetpoint();

    }
    public void setelevatorMotor(double volts){
        elevatorMotor.setVoltage(volts);
    }
    public double getelevatorEncoder() {
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void ResetelevatorEncoder() {
        elevatorMotor.setSelectedSensorPosition(0);
 
    }

    public double getAngleMotorEncoder(){
        return angleMotor.getSelectedSensorPosition();
    }

    public double encoderToDegrees(int encoder){
        return encoder / 2048 * 360;
    }

    public void setArmAngle(double pos){
        angleMotor.set(pid.calculate(angleMotor.getSelectedSensorPosition(), pos));
    }

}