package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandPneumatics implements Subsystem {

    private final PneumaticHub pneumaticHub;

    public CommandPneumatics(PneumaticHub pneumaticHub) {
        this.pneumaticHub = pneumaticHub;
    }

    private void updateTelemetry() {
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }


}
