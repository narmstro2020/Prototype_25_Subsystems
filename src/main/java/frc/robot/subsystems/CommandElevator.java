package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.ControlType.*;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static edu.wpi.first.units.Units.MetersPerSecond;


public class CommandElevator implements Subsystem {

    private final SparkMax motor;
    private final SimpleMotorFeedforward motorFeedforward;
    private final SlewRateLimiter slewRateLimiter;
    private final DCMotorSim motorSim;
    private final MutLinearVelocity motorSetpoint = MetersPerSecond.mutable(0.0);
    private final MutLinearVelocity motorVelocity = MetersPerSecond.mutable(0.0);
    private final SparkMaxSim sparkSim;
    private final double motorKs;

    public CommandElevator(final SparkMax motor,
                         final SimpleMotorFeedforward motorFeedforward,
                         final SlewRateLimiter slewRateLimiter,
                         final DCMotorSim motorSim,
                         final DCMotor dcMotor) {
        this.motor = motor;
        this.motorFeedforward = motorFeedforward;
        this.slewRateLimiter = slewRateLimiter;
        this.motorSim = motorSim;
        this.sparkSim = new SparkMaxSim(motor, dcMotor);
        motorKs = motorFeedforward.getKs();
    }

    private void updateTelemetry() {
        motorVelocity.mut_replace(motor.getEncoder().getVelocity(), MetersPerSecond);
    }

    private void applySetpoint() {
        double velocitySetpoint = slewRateLimiter.calculate(motorSetpoint.baseUnitMagnitude());
        double currentVelocity = motor.getEncoder().getVelocity();
        double arbFeedforward = motorFeedforward.calculateWithVelocities(currentVelocity, velocitySetpoint);
        arbFeedforward = MathUtil.clamp(arbFeedforward, -12.0, 12.0);
        motor.getClosedLoopController().setReference(velocitySetpoint, kVelocity, kSlot1, arbFeedforward, kVoltage);
    }

    private void stopMotor() {
        motorSetpoint.mut_replace(0.0, MetersPerSecond);
        motor.stopMotor();
    }

    private void stop() {
        stopMotor();
    }

    private void updateMotorSim() {
        double voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        if (voltage >= 0 && voltage <= motorKs) {
            voltage = 0;
        } else if (voltage < 0 && voltage >= -motorKs) {
            voltage = 0;
        }
        motorSim.setInputVoltage(voltage);
        motorSim.update(0.01);
        sparkSim.iterate(motorSim.getAngularVelocityRadPerSec(), 12.0, 0.01);

    }

    @Override
    public void periodic() {
        updateTelemetry();
        applySetpoint();
    }

    @Override
    public void simulationPeriodic() {
        updateMotorSim();
    }

    public Trigger motorAtSetpoint(LinearVelocity velocitySetpoint, LinearVelocity velocitySetpointTolerance) {
        return new Trigger(() -> motorVelocity.isNear(velocitySetpoint, velocitySetpointTolerance));
    }

    public Trigger motorsAtSetpoints(
            LinearVelocity motorSetpoint,
            LinearVelocity motorSetpointTolerance) {
        return motorAtSetpoint(motorSetpoint, motorSetpointTolerance);
    }

    public Command createStop() {
        return runOnce(this::stop).withName("Stop");
    }

    public Command createApplySetpoint(String name, LinearVelocity motorSetpoint) {
        return runOnce(() -> {
            this.motorSetpoint.mut_replace(motorSetpoint);
        }).withName(name);
    }



}







