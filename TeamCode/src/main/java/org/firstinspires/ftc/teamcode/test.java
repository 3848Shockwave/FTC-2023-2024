package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name = "OpenCV ", group = "Robot")
public class test extends OpMode {
OpenCvCamera cam = null ;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
       WebcamName camN = hardwareMap.get(WebcamName.class,"Webcam 1");
       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       cam = OpenCvCameraFactory.getInstance().createWebcam(camN, cameraMonitorViewId);
       cam.setPipeline(new Pipeline(telemetry));
       cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
               cam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
           }
           public void onError(int errorCode) {
               telemetry.addData("Error in OpenCV", errorCode);
              telemetry.update();
           }


       });
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}

class Pipeline extends OpenCvPipeline {
    private final Telemetry telemetry;

    public Pipeline(Telemetry telemetry) {
        // Use the telemetry object passed in to the constructor
        this.telemetry = telemetry;
    }

    Mat YCbCr = new Mat();
    Mat leftCrop ;
    Mat rightCrop;
    double leftAvgFin;
    double rightAvgFin ;
    Mat outPut = new Mat();
    Scalar rectColor = new Scalar(255, 0, 0);
    Mat cr = new Mat();  // Chrominance Red channel
    double avgCr;  // Average value from Chrominance Red channel
    Mat redmask = new Mat();
    Mat blueMask = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//        telemetry.addData("OpenCV", "Running");
//        telemetry.update();
        Rect leftR = new Rect(1, 1, 212, 479);
        Rect rightR = new Rect(214, 1, 426, 479);
        input.copyTo(outPut);
        Imgproc.rectangle(outPut, leftR, rectColor, 2);
        Imgproc.rectangle(outPut, rightR, rectColor, 2);
        leftCrop = YCbCr.submat(leftR);
        Core.extractChannel(YCbCr, cr, 1);
        // Threshold values for detecting red pixels (adjust these based on your environment)
        Scalar lowerRed = new Scalar(86, 112, 173);  // Lower bound for red in YCrCb
        Scalar upperRed = new Scalar(110, 110, 178);  // Upper bound for red in YCrCb

        // Create a binary mask based on the red threshold values

        Core.inRange(YCbCr, lowerRed, upperRed, redmask);

        // Calculate average value from the binary mask
        Scalar avgMask = Core.mean(redmask);

        // Threshold for detecting red pixels (adjust this threshold based on your environment)
        double redThreshold = 150;

        rightCrop = YCbCr.submat(rightR);
        Scalar leftAvg = Core.mean(leftCrop);
        Scalar rightAvg = Core.mean(rightCrop);
        leftAvgFin = leftAvg.val[0];
        rightAvgFin = rightAvg.val[0];
        if (avgMask.val[0] > redThreshold) {
            telemetry.addData("OpenCV", "Object Detected");


            if (leftAvgFin > rightAvgFin) {
                telemetry.addData("OpenCV", "Right");
            } else if (rightAvgFin > leftAvgFin) {
                telemetry.addData("OpenCV", "Left");
            } else {
                telemetry.addData("OpenCV", "None");
            }
        } else {
            telemetry.addData("OpenCV", "No Red Object");
        }
        Scalar lowerBlue = new Scalar(100, 0, 0);  // Lower bound for blue in YCrCb
        Scalar upperBlue = new Scalar(200, 255, 255);  // Upper bound for blue in YCrCb

        // Create a binary mask based on the blue threshold values

        Core.inRange(YCbCr, lowerBlue, upperBlue, blueMask);

        // Calculate average value from the blue binary mask
        Scalar avgBlueMask = Core.mean(blueMask);

        // Threshold for detecting blue pixels (adjust this threshold based on your environment)
        double blueThreshold = 150;

        // Check if the average blue mask value is above the threshold
        if (avgBlueMask.val[0] > blueThreshold) {
            telemetry.addData("OpenCV", " Object Detected");

            // Your existing code for figuring out which side it is on
            if (leftAvgFin > rightAvgFin) {
                telemetry.addData("OpenCV", "Right");
            } else if (rightAvgFin > leftAvgFin) {
                telemetry.addData("OpenCV", "Left");
            } else {
                telemetry.addData("OpenCV", "None");
            }
        } else {
            telemetry.addData("OpenCV", "No Blue Object");
        }


        return outPut;
    }
}
