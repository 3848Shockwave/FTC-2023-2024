package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal.Builder;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.util.Size;
import java.util.List;

@TeleOp(name = "Concept: AprilTag", group = "Concept")
public class VisionTest1 extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    // Variables to store live x, y, and z positions
    private double liveX, liveY, liveZ;

    @Override
    public void runOpMode() {
        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();

                // Save the live x, y, and z positions
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                if (!currentDetections.isEmpty()) {
                    AprilTagDetection detection = currentDetections.get(0);
                    liveX = detection.ftcPose.x;
                    liveY = detection.ftcPose.y;
                    liveZ = detection.ftcPose.z;

                    // Display live x, y, and z positions in telemetry
                    telemetry.addData("Live X", liveX);
                    telemetry.addData("Live Y", liveY);
                    telemetry.addData("Live Z", liveZ);
                }

                // Rest of your loop

                telemetry.update();
                sleep(20);
            }
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        AprilTagMetadata RedBack;
        AprilTagMetadata RedFront;
        AprilTagMetadata LeftBack;
        AprilTagMetadata LeftFront;


        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;

        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        RedBack = new AprilTagMetadata(372, "RB", 3.5, DistanceUnit.INCH);
        RedFront = new AprilTagMetadata(477, "RF", 3.5, DistanceUnit.INCH);
        LeftBack = new AprilTagMetadata(119, "LB", 3.5, DistanceUnit.INCH);
        LeftFront = new AprilTagMetadata(249, "LF", 3.5, DistanceUnit.INCH);

        myAprilTagLibraryBuilder.addTag(RedBack);
        myAprilTagLibraryBuilder.addTag(RedFront);
        myAprilTagLibraryBuilder.addTag(LeftBack);
        myAprilTagLibraryBuilder.addTag(LeftFront);


        myAprilTagLibrary = myAprilTagLibraryBuilder.build();
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);
        aprilTag = myAprilTagProcessorBuilder
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH,AngleUnit.DEGREES)
                .build();


        // The following default settings are available to un-comment and edit as needed.
        //.setDrawAxes(true)
        // .setDrawCubeProjection(true)
        // .setDrawTagOutline(true)

//                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH,AngleUnit.DEGREES)

        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        // ... these parameters are fx, fy, cx, cy.



        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        //Size sz = new Size(640,480);
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(sz);
        builder.setCameraResolution(new Size(640,480));
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        }

        // Update the telemetry
        telemetry.update();
    }

    // Method to get the live x, y, and z positions
    public double getLiveX() {
        return liveX;
    }

    public double getLiveY() {
        return liveY;
    }

    public double getLiveZ() {
        return liveZ;
    }
}
