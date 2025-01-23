package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters;
import static com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder;

public class CommandIntakeGen {

    public static CommandIntake create(){

        EncoderConfig encoderConfig0 = new EncoderConfig()
                .positionConversionFactor(2 * Math.PI)
                .velocityConversionFactor(2 * Math.PI / 60.0)
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(16);

        DCMotor dcMotor0 = DCMotor.getNeo550(1);


        double ks0 = RobotBase.isReal() ? 0.13511 : 0.0;
        double ka0 = 0.001258;
        double kv0 = 0.01041741445692405834123602814802;

        LinearSystem<N2, N1, N2> linearSystem0 = LinearSystemId.createDCMotorSystem(kv0, ka0);
        DCMotorSim dcMotorSim0 = new DCMotorSim(linearSystem0, dcMotor0);



        SimpleMotorFeedforward motor0Feedforward = new SimpleMotorFeedforward(ks0, kv0, ka0, 0.020);
        double maxVelocity0 = motor0Feedforward.maxAchievableVelocity(12.0, 0.0);
        double maxAcceleration0 = motor0Feedforward.maxAchievableAcceleration(12.0, 0.0);

        double kp0 = 0.0017206;


        ClosedLoopConfig closedLoopConfig0 = new ClosedLoopConfig()
                .p(kp0, ClosedLoopSlot.kSlot1)
                .i(0.0, ClosedLoopSlot.kSlot1)
                .d(0.0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                .feedbackSensor(kPrimaryEncoder);

        closedLoopConfig0.maxMotion.maxVelocity(maxVelocity0, ClosedLoopSlot.kSlot1);
        closedLoopConfig0.maxMotion.maxAcceleration(maxAcceleration0, ClosedLoopSlot.kSlot1);

        SparkMax sparkMax0 = new SparkMax(16, kBrushless);
        SparkBaseConfig sparkBaseConfig0 = new SparkMaxConfig()
                .apply(encoderConfig0)
                .apply(closedLoopConfig0);
        sparkMax0.configure(sparkBaseConfig0, kResetSafeParameters, kPersistParameters);


        return new CommandIntake(
                sparkMax0,
                motor0Feedforward,
                dcMotorSim0,
                dcMotor0);
    }
}
