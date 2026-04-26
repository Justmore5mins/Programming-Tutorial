package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static final double GearRatio = 10.71;
    
    public static final Distance TrackWidth = Inches.of(21.918500);
    public static final Distance WheelRadius = Inches.of(3);

    public static final double PositionConversionFactor = 1 / GearRatio;
    public static final double VelocityConversionFactor = PositionConversionFactor / 60;
    
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TrackWidth);

    public static final LinearVelocity MaxVelocity = MetersPerSecond.of(RPM.of(5676).div(GearRatio).in(RotationsPerSecond)*WheelRadius.times(2*Math.PI).in(Meters));
    public static final AngularVelocity MaxOmega = RadiansPerSecond.of(MaxVelocity.in(MetersPerSecond)/TrackWidth.div(2).in(Meters));

    public static final double SimMass = 28.0;
    public static final double SimMOI  = 1.3;

    public class LeftWheels {
        public static final int FrontMotor = 10;
        public static final int RearMotor = 11;
        public static final ClosedLoopConfig PID = new ClosedLoopConfig()
            .pid(0, 0, 0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        public static final FeedForwardConfig FF = new FeedForwardConfig()
            .sva(0, 0, 0);
        public static final MAXMotionConfig Motion = new MAXMotionConfig()
            .cruiseVelocity(0)
            .maxAcceleration(0);
    }

    public class RightWheels {
        public static final int FrontMotor = 12;
        public static final int RearMotor = 13;
        public static final ClosedLoopConfig PID = new ClosedLoopConfig()
            .pid(0, 0, 0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        public static final FeedForwardConfig FF = new FeedForwardConfig()
            .sva(0, 0, 0);
        public static final MAXMotionConfig Motion = new MAXMotionConfig()
            .cruiseVelocity(0)
            .maxAcceleration(0);
    }
}
