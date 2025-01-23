package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.revrobotics.spark.ClosedLoopSlot.*;
import static com.revrobotics.spark.SparkBase.ControlType.*;
import static com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits.kVoltage;
import static edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class CommandIntake implements Subsystem {


    public enum ControlMode {
        POSITION,
        VELOCITY
    }

    private final SparkMax sparkMax0;
    private final SparkMaxSim sparkMax0Sim;
    private final SimpleMotorFeedforward sparkMax0Feedforward;
    private final DCMotorSim dcMotorSim0;
    private final TrapezoidProfile positionProfile;
    private final State goalState = new State();
    private final State goalStateVelocity = new State();
    private final MutAngle intake0Position = Radians.mutable(0.0);
    private final MutAngularVelocity intake0Velocity = RadiansPerSecond.mutable(0.0);
    private final double maxAcceleration0;
    private State lastState = new State();
    private ControlMode controlMode = ControlMode.VELOCITY;


    public CommandIntake(final SparkMax sparkMax0,
                         final SimpleMotorFeedforward sparkMax0Feedforward,
                         final DCMotorSim dcMotorSim0,
                         final DCMotor dcMotor0) {
        this.sparkMax0 = sparkMax0;
        this.sparkMax0Feedforward = sparkMax0Feedforward;
        this.dcMotorSim0 = dcMotorSim0;
        this.sparkMax0Sim = new SparkMaxSim(sparkMax0, dcMotor0);
        double maxVelocity0 = sparkMax0Feedforward.maxAchievableVelocity(12.0, 0.0);
        this.maxAcceleration0 = sparkMax0Feedforward.maxAchievableAcceleration(12.0, 0.0);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                maxVelocity0, maxAcceleration0);
        positionProfile = new TrapezoidProfile(constraints);

        SmartDashboard.putNumber("Max Velocity", maxVelocity0);
        SmartDashboard.putNumber("Max Acceleration", maxAcceleration0);
    }

    private void updateTelemetry() {
        intake0Position.mut_setMagnitude(sparkMax0.getEncoder().getPosition());
        intake0Velocity.mut_setMagnitude(sparkMax0.getEncoder().getVelocity());
        SmartDashboard.putNumber("Position", intake0Position.baseUnitMagnitude());
        SmartDashboard.putNumber("Velocity", intake0Velocity.baseUnitMagnitude());
    }

    private void applyIntake0VelocitySetpoint() {
        double nextVelocity = goalState.velocity;
        double arbFeedforward = sparkMax0Feedforward.getKs() * Math.signum(nextVelocity);
        sparkMax0.getClosedLoopController().setReference(nextVelocity, kMAXMotionVelocityControl, kSlot1, arbFeedforward, kVoltage);
        lastState.velocity = nextVelocity;
        lastState.position = sparkMax0.getEncoder().getPosition();

    }

    public void applyIntake0PositionSetpoint() {
        double lastVelocity = lastState.velocity;
        lastState = positionProfile.calculate(0.010, lastState, goalState);
        double nextVelocity = lastState.velocity;
        double nextPosition = lastState.position;
        double arbFeedforward = sparkMax0Feedforward.calculateWithVelocities(lastVelocity, nextVelocity);
        sparkMax0.getClosedLoopController().setReference(nextPosition, kPosition, kSlot0, arbFeedforward, kVoltage);
    }

    private void stopIntake0() {
        controlMode = ControlMode.VELOCITY;
        goalState.velocity = 0.0;
        sparkMax0.stopMotor();
    }

    private void stop() {
        stopIntake0();
    }

    private void updateIntake0Sim() {
        double voltage = sparkMax0.getAppliedOutput() * (12.0);
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
            applyIntake0PositionSetpoint();
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
        return runOnce(this::stop).withName("Stop");
    }

    public Command createApplyVelocitySetpoint(String name, AngularVelocity motor0Setpoint) {
        return runOnce(() -> {
            controlMode = ControlMode.VELOCITY;
            goalState.velocity = motor0Setpoint.baseUnitMagnitude();
        }).withName(name);
    }

    public Command createApplyPositionSetpoint(String name, Angle motor0Setpoint) {
        return runOnce(() -> {
            controlMode = ControlMode.POSITION;
            goalState.position = motor0Setpoint.baseUnitMagnitude();
            goalState.velocity = 0.0;
        });
    }


}

