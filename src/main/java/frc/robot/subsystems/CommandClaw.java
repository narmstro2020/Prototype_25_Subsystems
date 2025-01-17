package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandClaw implements Subsystem {

    private final Solenoid solenoid;

    public CommandClaw(Solenoid solenoid) {
        this.solenoid = solenoid;
    }

    private void updateTelemetry(){

    }

    private void extend(){
        solenoid.set(true);
    }

    private void contract(){
        solenoid.set(false);
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public Trigger isExtended(){
        return new Trigger(solenoid::get);
    }

    public Trigger isContracted(){
        return new Trigger(() -> !solenoid.get());
    }

    public Command createExtend(){
        return run(this::extend).withName("Extend");
    }

    public Command createContract(){
        return run(this::contract).withName("Contract");
    }
}
