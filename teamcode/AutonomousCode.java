package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "DetectionAutoPractice", group =  "Auto")
public class AutonomousCode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        // Creating Instances of Each Subsystem of the Robot
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.lw = hardwareMap.dcMotor.get("lw");
        driveTrain.blw = hardwareMap.dcMotor.get("blw");
        driveTrain.rw = hardwareMap.dcMotor.get("rw");
        driveTrain.brw = hardwareMap.dcMotor.get("brw");

        ObjectDetector detector = new ObjectDetector(telemetry);
        detector.logiCam = hardwareMap.get(WebcamName.class, "logiCam");
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        detector.camera = OpenCvCameraFactory.getInstance().createWebcam(detector.logiCam);
        detector.camera.setPipeline(detector);
        detector.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        //camera.pauseViewport() and webcam.resumeViewport()

            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                detector.camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {

            }
        });


        waitForStart();
        switch (detector.getLocation()) {
            case LEFT:
                // do stuff
                break;
            case RIGHT:
                // do other stuff
                break;
            case MIDDLE:
                // also do stuff
                break;
        }
        detector.camera.stopStreaming();
        // run until the end of the match (driver presses STOP)
    }
}