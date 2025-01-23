package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Degrees;

public class CommandGyroscope implements Subsystem, Sendable {

    public final Pigeon2 pigeon2;

    public CommandGyroscope(Pigeon2 pigeon2) {
        this.pigeon2 = pigeon2;


    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "Yaw",
                () -> pigeon2.getYaw().getValue().in(Degrees),
                null);
        builder.addDoubleProperty(
                "Pitch",
                () -> pigeon2.getPitch().getValue().in(Degrees),
                null);
        builder.addDoubleProperty(
                "Roll",
                () -> pigeon2.getRoll().getValue().in(Degrees),
                null);
    }
}
