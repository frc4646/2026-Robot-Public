// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.vision;

import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.RobotState;
import frc.robot.configuration.Constants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.utils.FieldConstants_2026;

public class OrinVision extends SubsystemBase
{
    private static final String NAME = "OrinVision";

    private static final boolean ENABLE_SWERVE_POSE_RESET = true;

    private static final boolean OBJ_DETECT_EN = false;
    private static final double OBJ_DETECT_MIN_CONFIDENCE = 0.1;

    private static final double RECONNECT_TX_TIME_SEC = 2;

    private RobotState robotState;
    private CommandSwerveDrivetrain swerve;

    private DatagramChannel udpChannelRx;
    private DatagramChannel udpChannelRxObjDetect;
    private DatagramChannel udpChannelTx;

    private ByteBuffer bufferRx = ByteBuffer.allocate(2048);
    private ByteBuffer bufferRxObjDetect = ByteBuffer.allocate(2048);
    private ByteBuffer bufferTx = ByteBuffer.allocate(2048);

    /**
     * Connect to the Orin and get updated Robot Pose using a pair of UDP channels
     */
    public OrinVision(RobotState robotState, CommandSwerveDrivetrain swerve)
    {
        this.robotState = robotState;
        this.swerve = swerve;

        initChannels();
    }

    private void initChannels()
    {
        try
        {
            closeChannel(udpChannelRx);
            closeChannel(udpChannelRxObjDetect);
            closeChannel(udpChannelTx);

            udpChannelRx = DatagramChannel.open();
            udpChannelRx.configureBlocking(false);
            udpChannelRx.socket().bind(new InetSocketAddress(4648));

            // in theory we could disable this based on OBJ_DETECT_EN
            // but to prevent possible issues with the channels not being
            // created/destroyed correctly, just always create it
            // but only read from it if OBJ_DETECT_EN is true
            udpChannelRxObjDetect = DatagramChannel.open();
            udpChannelRxObjDetect.configureBlocking(false);
            udpChannelRxObjDetect.socket().bind(new InetSocketAddress(4649));

            udpChannelTx = DatagramChannel.open();
            udpChannelTx.connect(new InetSocketAddress(Constants.VISION_ORIN_IP, 4647));
            udpChannelTx.configureBlocking(false);

            // orin expects little endian
            bufferTx.order(ByteOrder.LITTLE_ENDIAN);
            bufferRx.order(ByteOrder.LITTLE_ENDIAN);
            bufferRxObjDetect.order(ByteOrder.LITTLE_ENDIAN);

        } catch (Exception e)
        {
            String msg = e.getMessage();
            if (msg != null)
            {
                // e.printStackTrace();
                System.err.println(NAME + ": " + msg);
            }
        }
    }

    private void closeChannel(DatagramChannel channel)
    {
        if (channel != null && channel.isOpen())
        {
            try
            {
                channel.close();
            } catch (Exception ignored)
            {

            }
        }
    }

    double lastTxTime = 0;

    @Override
    public void periodic()
    {
        double now = Timer.getFPGATimestamp();
        // if we haven't sent a message in a while
        if (now - lastTxTime > RECONNECT_TX_TIME_SEC)
        {
            Logger.recordOutput(NAME + "/reconnectTime", now);
            // close and reinit channels
            initChannels();
            // reset timer on attempt
            lastTxTime = now;
        }

        boolean rxConnected = receiveData();
        boolean rxObjDetected = false;
        if (OBJ_DETECT_EN)
        {
            rxObjDetected = receiveDataObjDetect();
        }
        boolean txConnected = sendStates(swerve.getState());

        // if connected reset timer
        if (txConnected)
        {
            lastTxTime = Timer.getFPGATimestamp();
        }

        Logger.recordOutput(NAME + "/rx/Connected", rxConnected);
        if (OBJ_DETECT_EN)
        {
            Logger.recordOutput(NAME + "/rxObjDetect/Detected", rxObjDetected);
        }
        Logger.recordOutput(NAME + "/tx/Connected", txConnected);
    }

    /**
     * Receive data from the vision system
     * 
     * @return true if new data was received
     */
    public boolean receiveData()
    {
        boolean hasUpdatedData = false;
        double x = 0.0;
        double y = 0.0;
        double rotationRadians = 0.0;
        double timestampPose_sec = 0.0;

        try
        {
            // continue to run get the last Orin message in the queue
            while (true)
            {
                bufferRx.clear();
                SocketAddress addr = udpChannelRx.receive(bufferRx);

                // until it says it has no more data
                if (addr == null)
                {
                    break;
                }

                // we have received a packet
                bufferRx.flip();

                // pull the data out of the buffer every time
                x = bufferRx.getDouble();
                y = bufferRx.getDouble();
                rotationRadians = bufferRx.getDouble();
                timestampPose_sec = bufferRx.getDouble() / 1000000.0; // time is in microsec

                // indicate that we have new data
                hasUpdatedData = true;
            }

            // if we got real data actually use it
            if (hasUpdatedData)
            {
                var timestamp_sec = Timer.getFPGATimestamp();

                Pose2d pose = new Pose2d(x, y, new Rotation2d(rotationRadians));

                Logger.recordOutput(NAME + "/rx/timeDiff", timestampPose_sec - timestamp_sec);
                Logger.recordOutput(NAME + "/rx/Pose", pose);

                Pose2d center = new Pose2d(FieldConstants_2026.fieldLength / 2.0, FieldConstants_2026.fieldWidth / 2.0, new Rotation2d());

                // if the timestamp is within 1 second of the current time, update the vision pose
                if (Util.epsilonEquals(timestampPose_sec, timestamp_sec, 1.0))
                {
                    // sanity check - valid poses shouldn't be 100m off 0
                    if (Math.abs(pose.getX()) < 100 && Math.abs(pose.getY()) < 100 && pose.getTranslation().getNorm() < 100)
                    {
                        // robotState.vision.setPose(pose);
                        // swerve.addVisionMeasurement(pose.value, timestampPose_sec);
                        // if (!(pose.minus(robotState.swerve.getRobotPose()).getTranslation().getNorm() > 1.0 && RobotContainer.enabled))
                        {
                            if (ENABLE_SWERVE_POSE_RESET && (pose.minus(center).getTranslation().getNorm() > 1.0))
                            {
                                swerve.resetTranslation(pose.getTranslation());
                            }
                        }
                    }
                }

                // robotState.visionSeesTag = visibleTags >= 1.0;
            }
        } catch (Exception e)
        {
            String msg = e.getMessage();
            if (msg != null)
            {
                // e.printStackTrace();
                System.err.println(NAME + ": " + msg);
            }
        }

        return hasUpdatedData;
    }

    /**
     * Receive data from the object detection system
     * 
     * @return true if new data was received
     */
    public boolean receiveDataObjDetect()
    {
        boolean hasUpdatedData = false;
        double confidence = 0.0;
        double x = 0.0;
        double y = 0.0;
        double width = 0.0;
        double height = 0.0;
        double timestampPose_sec = 0.0;

        try
        {
            // continue to run get the last Orin message in the queue
            while (true)
            {
                bufferRxObjDetect.clear();
                SocketAddress addr = udpChannelRxObjDetect.receive(bufferRxObjDetect);

                // until it says it has no more data
                if (addr == null)
                {
                    break;
                }

                // we have received a packet
                bufferRxObjDetect.flip();

                // pull the data out of the buffer every time
                confidence = bufferRxObjDetect.getDouble();
                x = bufferRxObjDetect.getDouble();
                y = bufferRxObjDetect.getDouble();
                width = bufferRxObjDetect.getDouble();
                height = bufferRxObjDetect.getDouble();
                timestampPose_sec = bufferRxObjDetect.getDouble() / 1000000.0; // time is in microsec

                // indicate that we have new data
                hasUpdatedData = true;
            }

            // if we got real data actually use it
            if (hasUpdatedData)
            {
                var timestamp_sec = Timer.getFPGATimestamp();
                Pose2d pose = new Pose2d(x, y, new Rotation2d(0));

                Logger.recordOutput(NAME + "/rxObjDetect/timeDiff", timestampPose_sec - timestamp_sec);
                Logger.recordOutput(NAME + "/rxObjDetect/Pose", pose);
                Logger.recordOutput(NAME + "/rxObjDetect/Confidence", confidence);
                Logger.recordOutput(NAME + "/rxObjDetect/Width", width);
                Logger.recordOutput(NAME + "/rxObjDetect/Height", height);

                // if (confidence > OBJ_DETECT_MIN_CONFIDENCE && Util.epsilonEquals(timestampPose_sec, timestamp_sec, 1.0))
                // {
                // // pose shouldn't change much too quickly
                // // if it's been a while since we last got a valid pose, take any new pose
                // //
                // double changeInObjPose = robotState.objectDetection.getPose().minus(pose).getTranslation().getNorm();

                // if (changeInObjPose < .15)
                // {
                // robotState.objectDetection.setConfidence(confidence);
                // robotState.objectDetection.setPose(pose);
                // robotState.objectDetection.setWidth(width);
                // robotState.objectDetection.setHeight(height);
                // robotState.objectDetection.setTimestampPose_sec(timestampPose_sec);
                // robotState.objectDetection.setIsValid(true);
                // }
                // else if (Math.abs(timestampPose_sec - robotState.objectDetection.getTimestampPose_sec()) > .5)
                // {
                // // it's been a while since we've seen the object, so use the new pose
                // robotState.objectDetection.setConfidence(confidence);
                // robotState.objectDetection.setPose(pose);
                // robotState.objectDetection.setWidth(width);
                // robotState.objectDetection.setHeight(height);
                // robotState.objectDetection.setTimestampPose_sec(timestampPose_sec);
                // robotState.objectDetection.setIsValid(true);
                // }
                // }
                // else
                // {
                // // if the confidence is low, we don't have a valid object
                // hasUpdatedData = false;
                // robotState.objectDetection.setConfidence(confidence);
                // robotState.objectDetection.setIsValid(false);
                // }
            }
        } catch (Exception e)
        {
            String msg = e.getMessage();
            if (msg != null)
            {
                // e.printStackTrace();
                System.err.println(NAME + "ObjDetect: " + msg);
            }
        }
        // Logger.recordOutput(NAME + "/rxObjDetect/IsValid", robotState.objectDetection.getIsValid());

        return hasUpdatedData;
    }

    /**
     * Send the current swerve state to the vision system
     * 
     * @param state
     * @return true if the data was sent
     */
    public boolean sendStates(SwerveDriveState state)
    {
        // if we have nothing connected, don't send
        if (!udpChannelTx.isConnected())
        {
            return false;
        }

        Pose2d pose = state.Pose;
        ChassisSpeeds speeds = state.Speeds;

        try
        {
            bufferTx.clear();

            // swerve pose
            bufferTx.putDouble(pose.getX());
            bufferTx.putDouble(pose.getY());
            bufferTx.putDouble(pose.getRotation().getRadians());

            // swerve speeds
            bufferTx.putDouble(speeds.vxMetersPerSecond);
            bufferTx.putDouble(speeds.vyMetersPerSecond);
            bufferTx.putDouble(speeds.omegaRadiansPerSecond);

            // timestamp from swerve in the FPGA base time
            bufferTx.putDouble(state.Timestamp - Utils.getCurrentTimeSeconds() + Timer.getFPGATimestamp());

            // send the data
            bufferTx.flip();
            udpChannelTx.write(bufferTx);

            return true;

        } catch (Exception e)
        {
            String msg = e.getMessage();
            if (msg != null)
            {
                // e.printStackTrace();
                System.err.println(NAME + ": " + msg);
            }
        }
        return false;
    }

}
