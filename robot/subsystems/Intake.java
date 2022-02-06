package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public final Solenoid rightIntakeFlipper = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    public final Solenoid leftIntakeFlipper = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    public Intake() {}
}
