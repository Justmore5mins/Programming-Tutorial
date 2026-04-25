package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DataLog implements Subsystem{
    private Drivetrain drivetrain = Drivetrain.getInstance();

    private StructPublisher<Pose2d> CurrentPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/CurrentPose", Pose2d.struct).publish();
    private StructPublisher<ChassisSpeeds> CurrentSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/CurrentSpeeds", ChassisSpeeds.struct).publish();
    private StructPublisher<DifferentialDriveWheelSpeeds> CurrentWheelSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/CurrentWheelSpeeds", DifferentialDriveWheelSpeeds.struct).publish();
    private StructPublisher<DifferentialDriveWheelPositions> CurrentPosition = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/CurrentPosition", DifferentialDriveWheelPositions.struct).publish();
    private DoublePublisher BatteryVoltage = NetworkTableInstance.getDefault().getDoubleTopic("System/BatteryVoltage").publish();
    private DoublePublisher CANUtil = NetworkTableInstance.getDefault().getDoubleTopic("System/CAN/CANUtil").publish();
    private BooleanPublisher RSLState = NetworkTableInstance.getDefault().getBooleanTopic("System/RSLState").publish();
    @Override
    public void periodic(){
        CurrentPose.accept(drivetrain.PoseEstimator.getEstimatedPosition());
        CurrentSpeeds.accept(Constants.kinematics.toChassisSpeeds(drivetrain.getSpeeds()));
        CurrentWheelSpeeds.accept(drivetrain.getSpeeds());
        CurrentPosition.accept(drivetrain.getPosition());
        BatteryVoltage.accept(RobotController.getBatteryVoltage());
        CANUtil.accept(RobotController.getCANStatus().percentBusUtilization);
        RSLState.accept(RobotController.getRSLState());
    }
}
