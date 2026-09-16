package org.firstinspires.ftc.teamcode.lib.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.vision.AprilTagSmartDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@TeleOp(name = "TestAprilTag", group = "Test")
public class TestAprilTag extends LinearOpMode {
    OpenCvWebcam webcam;
    AprilTagSmartDetection pipeline = new AprilTagSmartDetection();
    boolean cameraOK = true;

    private void initDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setMillisecondsPermissionTimeout(3000); //3000
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                try {
                    Thread.sleep(1000); // always wait before pressing start
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                webcam.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("camera failed to open:", errorCode);
                telemetry.update();
                cameraOK = false;
            }
        });
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initDetection();

        waitForStart();
        pipeline.begin();

        while(isStarted() && !isStopRequested()){
            telemetry.addData("translation:", pipeline.tvec.dump());
            //telemetry.addData("t2:", pipeline.tvec2.dump());
            telemetry.addData("rotation:", pipeline.rvec.dump());
            //telemetry.addData("r2:", pipeline.rvec2.dump());
            telemetry.addData("coordsH:", pipeline.coordsH.dump());
            telemetry.addData("theta:", pipeline.theta);
            telemetry.addData("id:", pipeline.getTagId());
            telemetry.addData("debug", pipeline.debug);
            telemetry.update();
        }

        pipeline.kill();
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
