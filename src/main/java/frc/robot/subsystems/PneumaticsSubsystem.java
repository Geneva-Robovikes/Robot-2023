package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PneumaticsSubsystem extends SubsystemBase {
    Compressor phCompressor;
    DoubleSolenoid exampleDoublePH;
    boolean cubeMode;

    public PneumaticsSubsystem() {
        phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
        exampleDoublePH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
        exampleDoublePH.set(kReverse);
    }

    
    public boolean getPressureSwitch() {
        return phCompressor.getPressureSwitchValue();
    }

    public void setSolenoid(Value value) {
        exampleDoublePH.set(value);
    }

    public void switchMode() {
        if (cubeMode) {
            cubeMode = false;
        } else {
            cubeMode = true;
        }
    }

    public boolean getMode() {
        return cubeMode;
    }

    public void setCubeMode(boolean cubeMode) {
        this.cubeMode = cubeMode;
    }


    public CommandBase setCubeModeCommand() {
        return runOnce(() -> switchMode());
    }
}
