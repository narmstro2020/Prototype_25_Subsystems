package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.ControlType.*;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class CommandIntake implements Subsystem {


    private final SlewRateLimiter velocityProfile;

    public enum ControlMode {
        VELOCITY,
        STOP
    }

    private final SparkMax sparkMax0;
    private final SparkMaxSim sparkMax0Sim;
    private final SimpleMotorFeedforward sparkMax0Feedforward;
    private final DCMotorSim dcMotorSim0;
    private final MutAngularVelocity intake0Velocity = RadiansPerSecond.mutable(0.0);
    private final MutAngularVelocity angularVelocityGoal = RadiansPerSecond.mutable(0.0);
    private final MutAngularVelocity angularVelocityState = RadiansPerSecond.mutable(0.0);
    private ControlMode controlMode = ControlMode.STOP;


    public CommandIntake(final SparkMax sparkMax0,
                         final SimpleMotorFeedforward sparkMax0Feedforward,
                         final DCMotorSim dcMotorSim0,
                         final DCMotor dcMotor0) {
        this.sparkMax0 = sparkMax0;
        this.sparkMax0Feedforward = sparkMax0Feedforward;
        this.dcMotorSim0 = dcMotorSim0;
        this.sparkMax0Sim = new SparkMaxSim(sparkMax0, dcMotor0);
        double maxVelocity0 = sparkMax0Feedforward.maxAchievableVelocity(12.0, 0.0);
        double maxAcceleration0 = sparkMax0Feedforward.maxAchievableAcceleration(12.0, 0.0);

        velocityProfile = new SlewRateLimiter(maxAcceleration0);

        SmartDashboard.putNumber("Max Velocity", maxVelocity0);
        SmartDashboard.putNumber("Max Acceleration", maxAcceleration0);
    }

    private void updateTelemetry() {
        intake0Velocity.mut_setMagnitude(sparkMax0.getEncoder().getVelocity());
        SmartDashboard.putNumber("Velocity", intake0Velocity.baseUnitMagnitude());
    }

    private void applyIntake0VelocitySetpoint() {
        double currentVelocity = angularVelocityState.baseUnitMagnitude();
        double nextVelocity = velocityProfile.calculate(angularVelocityGoal.baseUnitMagnitude());
        angularVelocityState.mut_setMagnitude(nextVelocity);
        double arbFeedforward = sparkMax0Feedforward.calculateWithVelocities(currentVelocity, nextVelocity);
        sparkMax0.getClosedLoopController().setReference(nextVelocity, kVelocity, kSlot1, arbFeedforward, kVoltage);
    }

    private void stopIntake0() {
        sparkMax0.stopMotor();
        angularVelocityState.mut_replace(sparkMax0.getEncoder().getVelocity(), RadiansPerSecond);
    }

    private void stop() {
        stopIntake0();
    }

    private void updateIntake0Sim() {
        double voltage = sparkMax0Sim.getAppliedOutput() * sparkMax0Sim.getBusVoltage();
        dcMotorSim0.setInputVoltage(voltage);
        dcMotorSim0.update(0.01);
        sparkMax0Sim.iterate(dcMotorSim0.getAngularVelocityRadPerSec(), 12.0, 0.01);
    }

    @Override
    public void periodic() {
        updateTelemetry();
        if (controlMode == ControlMode.VELOCITY) {
            applyIntake0VelocitySetpoint();
        } else {
            stop();
        }
    }

    @Override
    public void simulationPeriodic() {
        updateIntake0Sim();
    }

    // triggers factories
    public Trigger motor0AtSetpoint(AngularVelocity velocitySetpoint, AngularVelocity velocitySetpointTolerance) {
        return new Trigger(() -> intake0Velocity.isNear(velocitySetpoint, velocitySetpointTolerance));
    }

    public Trigger motorsAtSetpoints(
            AngularVelocity motor0Setpoint,
            AngularVelocity motor0SetpointTolerance) {
        return motor0AtSetpoint(motor0Setpoint, motor0SetpointTolerance);
    }

    public Command createStop() {
        return runOnce(() -> {
            controlMode = ControlMode.STOP;
            angularVelocityGoal.mut_replace(0.0, RadiansPerSecond);
        }).withName("Stop");
    }

    public Command createApplyVelocitySetpoint(String name, AngularVelocity motor0Setpoint) {
        return runOnce(() -> {
            controlMode = ControlMode.VELOCITY;
            angularVelocityGoal.mut_replace(motor0Setpoint.baseUnitMagnitude(), RadiansPerSecond);
        }).withName(name);
    }


}

