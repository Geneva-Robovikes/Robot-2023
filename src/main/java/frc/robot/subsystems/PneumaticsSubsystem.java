package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PneumaticsSubsystem extends SubsystemBase {
    Compressor phCompressor;
    DoubleSolenoid exampleDoublePH;

    public PneumaticsSubsystem() {
        phCompressor = new Compressor(0, PneumaticsModuleType.REVPH);
        exampleDoublePH = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
        exampleDoublePH.set(kReverse);
    }

    
    public boolean getPressureSwitch() {
        return phCompressor.getPressureSwitchValue();
    }

    public void setSolenoid(Value value) {
        exampleDoublePH.set(value);
    }
}
