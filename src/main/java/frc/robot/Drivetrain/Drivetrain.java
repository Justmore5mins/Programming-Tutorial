package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Constants.LeftWheels;
import frc.robot.Drivetrain.Constants.RightWheels;

/**
 * see also:
 *  https://en.wikipedia.org/wiki/Linear–quadratic_regulator
 */


public class Drivetrain implements Subsystem {

    /**
     * defines the motors and other essential parts
     */

    /**
     * This expression equals to
     * public SparkMax FrontLeftMotor, FrontRightMotor, RearLeftMotor, RearRightMotor, but the List iterator can make the program more concisely and readable
     */
    public List<SparkMax> motors;
    public SparkMaxSim leftSimMotor, rightSimMotor;
    
    public List<RelativeEncoder> encoders;
    public SparkRelativeEncoderSim leftSimEncoder, rightSimEncoder;

    public SparkClosedLoopController LeftPID, RightPID;
    public AHRS gyro; //using the NavX gyroscope, putting it on the RoboRIO MXP port
    public DifferentialDrivePoseEstimator PoseEstimator;
    private static Drivetrain inst;
    
    private SparkMaxConfig FrontLeftConfig, FrontRightConfig, BackLeftConfig, BackRightConfig;

    private DifferentialDrivetrainSim driveSim;

    private Drivetrain(){
        //initialize the variables we just defined

        motors = List.of(
            new SparkMax(LeftWheels.FrontMotor, MotorType.kBrushless),
            new SparkMax(LeftWheels.RearMotor, MotorType.kBrushless),
            new SparkMax(RightWheels.FrontMotor, MotorType.kBrushless),
            new SparkMax(RightWheels.RearMotor, MotorType.kBrushless)
        );
        encoders = motors.stream().map(SparkMax::getEncoder).toList();

        LeftPID = motors.get(0).getClosedLoopController();
        RightPID = motors.get(2).getClosedLoopController();

        gyro = new AHRS(NavXComType.kMXP_SPI);

        FrontLeftConfig = new SparkMaxConfig();
        FrontRightConfig = new SparkMaxConfig();
        BackLeftConfig = new SparkMaxConfig();
        BackRightConfig = new SparkMaxConfig();

        FrontLeftConfig
            .idleMode(IdleMode.kBrake) 
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(40);
        FrontLeftConfig.closedLoop.apply(LeftWheels.PID).apply(LeftWheels.FF).apply(LeftWheels.Motion); //applying the clsed loop control configuration
        FrontLeftConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor) //Convert to mechanism rotations
            .velocityConversionFactor(Constants.VelocityConversionFactor); //Convert to mechanism rot/s
        BackLeftConfig
            .follow(LeftWheels.FrontMotor);

        FrontRightConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(40);
        FrontRightConfig.closedLoop.apply(RightWheels.PID).apply(RightWheels.FF).apply(RightWheels.Motion);
        FrontRightConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);
        BackRightConfig
            .follow(RightWheels.FrontMotor);

        motors.get(0).configure(FrontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motors.get(1).configure(BackLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motors.get(2).configure(FrontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motors.get(3).configure(BackRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        if(RobotBase.isSimulation()) simInit();

        PoseEstimator = new DifferentialDrivePoseEstimator(Constants.kinematics, gyro.getRotation2d(), getPosition().leftMeters, getPosition().rightMeters, new Pose2d());

        autoInit();
    }

    public DifferentialDriveWheelPositions getPosition(){
        if(RobotBase.isReal()) {
            return new DifferentialDriveWheelPositions(
                Meters.of(encoders.get(0).getPosition() * Constants.WheelRadius.times(2 * Math.PI).in(Meters)),
                Meters.of(encoders.get(2).getPosition() * Constants.WheelRadius.times(2 * Math.PI).in(Meters))
            );
        }
        else {
            return new DifferentialDriveWheelPositions(
                Meters.of(leftSimEncoder.getPosition() * Constants.WheelRadius.times(2 * Math.PI).in(Meters)),
                Meters.of(rightSimEncoder.getPosition() * Constants.WheelRadius.times(2 * Math.PI).in(Meters))
            );
        }
    }

    public DifferentialDriveWheelSpeeds getSpeeds(){
        if(RobotBase.isReal()) {
            return new DifferentialDriveWheelSpeeds(
                MetersPerSecond.of(encoders.get(0).getVelocity() * Constants.WheelRadius.times(2 * Math.PI).in(Meters)),
                MetersPerSecond.of(encoders.get(2).getVelocity() * Constants.WheelRadius.times(2 * Math.PI).in(Meters))
            );
        }
        else {
            return new DifferentialDriveWheelSpeeds(
                MetersPerSecond.of(leftSimEncoder.getVelocity() * Constants.WheelRadius.times(2 * Math.PI).in(Meters)),
                MetersPerSecond.of(rightSimEncoder.getVelocity() * Constants.WheelRadius.times(2 * Math.PI).in(Meters))
            );
        }
    }

    public Pose2d getCurrentPose() {
        return PoseEstimator.getEstimatedPosition();
    }

    public Command drive(Supplier<LinearVelocity> vx, Supplier<LinearVelocity>vy, Supplier<AngularVelocity> omega){
        return run(() -> drive(new ChassisSpeeds(vx.get(), vy.get(), omega.get())));
    }

    private void drive(ChassisSpeeds speeds) {
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.01);
        DifferentialDriveWheelSpeeds targetSpeeds = Constants.kinematics.toWheelSpeeds(discretizedSpeeds);

        targetSpeeds.desaturate(Constants.MaxVelocity);

        if(RobotBase.isReal()) {
            LeftPID.setSetpoint(targetSpeeds.leftMetersPerSecond, ControlType.kMAXMotionVelocityControl);
            RightPID.setSetpoint(targetSpeeds.rightMetersPerSecond, ControlType.kMAXMotionVelocityControl);
        }
        else {
            double leftSpeed = targetSpeeds.leftMetersPerSecond / Constants.MaxVelocity.in(MetersPerSecond);
            double rightSpeed = targetSpeeds.rightMetersPerSecond / Constants.MaxVelocity.in(MetersPerSecond);

            leftSimMotor.setAppliedOutput(leftSpeed);
            rightSimMotor.setAppliedOutput(rightSpeed);
        }
    }

    public void resetPose(Pose2d pose){
            PoseEstimator.resetPose(pose);
        }


    @Override
    public void periodic(){
        PoseEstimator.update(gyro.getRotation2d(), getPosition());
    }

    @Override
    public void simulationPeriodic() {
        driveSim.setInputs(
            leftSimMotor.getAppliedOutput() * RobotController.getBatteryVoltage(),
            rightSimMotor.getAppliedOutput() * RobotController.getBatteryVoltage()
        );

        driveSim.update(0.02);

        leftSimEncoder.setPosition(driveSim.getLeftPositionMeters());
        rightSimEncoder.setPosition(driveSim.getRightPositionMeters());

        leftSimEncoder.setVelocity(driveSim.getLeftVelocityMetersPerSecond());
        rightSimEncoder.setVelocity(driveSim.getRightVelocityMetersPerSecond());

        gyro.setAngleAdjustment(-driveSim.getHeading().getDegrees());
    }

    private void autoInit(){
        try{
            AutoBuilder.configure(
            () -> PoseEstimator.getEstimatedPosition(), 
            this::resetPose, 
            () -> Constants.kinematics.toChassisSpeeds(getSpeeds()), 
            (speeds, ff) -> drive(speeds), 
            new PPLTVController(
                VecBuilder.fill(1.0,1.0,1.0), 
                VecBuilder.fill(1.0,1.0),
                0.02
            ), 
            RobotConfig.fromGUISettings(), 
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
            this); 
       }catch(Exception e){
        DriverStation.reportError(e.getMessage(), e.getStackTrace());
       }
    }

    private void simInit() {
        driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),
            Constants.GearRatio,
            Constants.SimMOI,
            Constants.SimMass,
            Constants.WheelRadius.in(Meters),
            Constants.kinematics.trackWidthMeters,
            null
        );

        leftSimMotor = new SparkMaxSim(motors.get(0), DCMotor.getNEO(2));
        rightSimMotor = new SparkMaxSim(motors.get(2), DCMotor.getNEO(2));

        leftSimEncoder = leftSimMotor.getRelativeEncoderSim();
        rightSimEncoder = rightSimMotor.getRelativeEncoderSim();

        leftSimMotor.useDriverStationEnable();
        rightSimMotor.useDriverStationEnable();
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }
}
