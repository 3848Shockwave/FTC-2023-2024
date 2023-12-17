package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@TeleOp(name = "OpenCV2 ", group = "Robot")
public class test2 extends OpMode {
OpenCvCamera cam = null ;
int pos = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
       WebcamName camN = hardwareMap.get(WebcamName.class,"Webcam 1");
       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       cam = OpenCvCameraFactory.getInstance().createWebcam(camN, cameraMonitorViewId);
       cam.setPipeline(new PipelineReds2(telemetry,pos));

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
    public void setPos(int pos) {
        this.pos = pos;
    }
}

class PipelineReds2 extends OpenCvPipeline {
    private final Telemetry telemetry;
    private int pos;
    public PipelineReds2(Telemetry telemetry, int pos) {
        this.telemetry = telemetry;
        this.pos = pos;
    }

    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    Mat centerCrop;
    double leftAvgFin;
    double centerAvgFin;
    double rightAvgFin;
    Mat output = new Mat();
    Scalar rectColor = new Scalar(255, 0, 0);
    test2 test = new test2();
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

        Rect leftR = new Rect(1, 1, 212, 479);
        Rect centerR = new Rect(214,1,212,479); // Center
        Rect rightR = new Rect(428, 1, 212, 479);

        input.copyTo(output);
        Imgproc.rectangle(output, leftR, rectColor, 2);
        Imgproc.rectangle(output, centerR, rectColor, 2);
        Imgproc.rectangle(output, rightR, rectColor, 2);

        leftCrop = YCbCr.submat(leftR);
        centerCrop = YCbCr.submat(centerR);

        rightCrop = YCbCr.submat(rightR);

        Scalar leftAvg = Core.mean(leftCrop);
        Scalar centerAvg = Core.mean(centerCrop);
        Scalar rightAvg = Core.mean(rightCrop);

        leftAvgFin = leftAvg.val[0];
        centerAvgFin = centerAvg.val[0];
        rightAvgFin = rightAvg.val[0];

        if (leftAvgFin > rightAvgFin && leftAvgFin > centerAvgFin) {
            telemetry.addData("OpenCV", "Right");
            pos = 3;
            test.setPos(pos);
        }
        if (rightAvgFin > leftAvgFin && rightAvgFin > centerAvgFin) {
            telemetry.addData("OpenCV", "Left");
            pos=1;
            test.setPos(pos);
        }
        if (centerAvgFin > leftAvgFin && centerAvgFin > rightAvgFin) {
            telemetry.addData("OpenCV", "Center");
            pos=2;
            test.setPos(pos);
        }
        else {
            telemetry.addData("OpenCV", "None");
        }

        telemetry.update();
        return output;
    }
}

