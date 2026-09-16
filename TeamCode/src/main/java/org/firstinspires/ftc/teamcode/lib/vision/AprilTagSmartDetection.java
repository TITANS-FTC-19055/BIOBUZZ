package org.firstinspires.ftc.teamcode.lib.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.apriltag.ApriltagDetectionJNI;
import org.openftc.easyopencv.OpenCvPipeline;

// is this actually even smart ???
public class AprilTagSmartDetection extends OpenCvPipeline {
    public final long detectorPtr;
    private static final int width_res = 640, height_res = 480;
    private boolean kill = false, start = false;
    public int debug = -1;

    public static final double TAGSIZE = 0.1016, FX = 822.317, FY = 822.317, CX = 319.495, CY = 242.502;

    public AprilTagSmartDetection(){
        this.detectorPtr = AprilTagDetectorJNI.createApriltagDetector("tag36h11", 2F, 2);

        coordsH.put(0, 0, new double[]{0, 0, 0, 1});

        if(this.detectorPtr == 0){
            this.kill = true;
        }
    }

    public void begin(){
        this.start = true;
    }

    public void kill(){
        this.kill = true;
        AprilTagDetectorJNI.releaseApriltagDetector(detectorPtr);
    }

    private Mat colorStream = new Mat();
    private long tagsPtrZarr;
    private MatOfPoint2f crazyMat = new MatOfPoint2f();
    private int tagId = -1;

    private int detectAllTags(Mat input){
        if(input.empty()){
            return 0;
        }

        input.copyTo(colorStream);
        //if(this.cropRect != null){
        //    Rect roi = new Rect(cropRect[0], cropRect[1], cropRect[2], cropRect[3]);
        //    input = input.submat(roi);
        //}
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);

        // tagsPtr will be a zarray_t *
        tagsPtrZarr = AprilTagDetectorJNI.runApriltagDetector(detectorPtr, input.dataAddr(), width_res, height_res);

        if(tagsPtrZarr == 0){
            debug = 0;
            return 0;
        }

        // now we have a normal array of apriltag_detection_t *
        long[] tagsPtrsArray = ApriltagDetectionJNI.getDetectionPointers(tagsPtrZarr);

        if(tagsPtrsArray.length == 0){
            debug = 1;
            return 0;
        }

        if(tagsPtrsArray[0] == 0){
            debug = 2;
            return 0;
        }

        tagId = ApriltagDetectionJNI.getId(tagsPtrsArray[0]);
        double[][] c = ApriltagDetectionJNI.getCorners(tagsPtrsArray[0]);

        crazyMat.fromArray(
                new Point(c[0][0], c[0][1]),
                new Point(c[1][0], c[1][1]),
                new Point(c[2][0], c[2][1]),
                new Point(c[3][0], c[3][1])
                );

        ApriltagDetectionJNI.freeDetectionList(tagsPtrZarr);

        debug = 3;
        return 1;
    }

    // real frame of reference: O is at the center of the tag
    private static final MatOfPoint3f mat3dapriltag = new MatOfPoint3f(
            new Point3(-TAGSIZE/2, +TAGSIZE/2, 0d),
            new Point3(+TAGSIZE/2, +TAGSIZE/2, 0d),
            new Point3(+TAGSIZE/2, -TAGSIZE/2, 0d),
            new Point3(-TAGSIZE/2, -TAGSIZE/2, 0d)
    );
    private static final Mat intrinsicParams = new Mat(3, 3, CvType.CV_32F);
    static {
        intrinsicParams.put(0, 0, new double[]{FX, 0, CX, 0, FY, CY, 0, 0, 1});
    }

    private static final MatOfDouble distCoeffs = new MatOfDouble();
    static {
        distCoeffs.fromArray(new double[] {-0.0449369, 1.17277, 0, 0, -1.33244, 0, 0, 0});
    }

    public Mat rvec = new Mat(1, 3, CvType.CV_32F), tvec = new Mat(1, 3, CvType.CV_32F);
    private void computeTvecRvec(MatOfPoint2f mat2dcameraframe){
        Calib3d.solvePnP(
                mat3dapriltag,
                mat2dcameraframe,
                intrinsicParams,
                distCoeffs,
                rvec, tvec,
                false, Calib3d.SOLVEPNP_IPPE_SQUARE
        );

        Calib3d.solvePnPRefineLM(mat3dapriltag, mat2dcameraframe, intrinsicParams, distCoeffs, rvec, tvec);
    }

    public static Mat buildTransformationMatrix(Mat _rvec, Mat _tvec){
        // rodrigues to get R = RxRyRz
        Mat rmat = new Mat(3, 3, CvType.CV_32F);
        Calib3d.Rodrigues(_rvec, rmat);

        // build transform matrix
            /*
            T = [ r11 r12 r13 x ]
                [ r21 r22 r23 y ]
                [ r31 r32 r33 z ]
                [ 0   0   0   1 ]
             */
        Mat transform = new Mat(4, 4, CvType.CV_32F);
        transform.put(0, 0, rmat.get(0, 0));
        transform.put(0, 1, rmat.get(0, 1));
        transform.put(0, 2, rmat.get(0, 2));
        transform.put(0, 3, _tvec.get(0, 0));
        transform.put(1, 0, rmat.get(1, 0));
        transform.put(1, 1, rmat.get(1, 1));
        transform.put(1, 2, rmat.get(1, 2));
        transform.put(1, 3, _tvec.get(0, 1));
        transform.put(2, 0, rmat.get(2, 0));
        transform.put(2, 1, rmat.get(2, 1));
        transform.put(2, 2, rmat.get(2, 2));
        transform.put(2, 3, _tvec.get(0, 2));
        transform.put(3, 0, 0);
        transform.put(3, 1, 0);
        transform.put(3, 2, 0);
        transform.put(3, 3, 1);


        return transform;
    }

    public static final Mat originH = new Mat(4, 1, CvType.CV_32F);
    static {
        originH.put(0, 0, new double[]{0, 0, 0, 1});
    }

    public static final double YAW = 0, PITCH = 0, ROLL = 0;
    public static final double XCAM = 0, YCAM = 0, ZCAM = 0;
    public static final Mat rvecCamera = new Mat(1, 3, CvType.CV_32F);
    static{
        rvecCamera.put(0, 0, new double[]{PITCH, YAW, ROLL});
    }
    public static final Mat tvecCamera = new Mat(1, 3, CvType.CV_32F);
    static{
        tvecCamera.put(0, 0, new double[]{XCAM, YCAM, ZCAM});
    }
    // transformation matrix from camera frame to robot frame
    public static final Mat transformCamera2Robot = buildTransformationMatrix(rvecCamera, tvecCamera).inv();

    public static Mat getRvecFromRmat(Mat _rmat){
        Mat _rvec = new Mat(1, 3, CvType.CV_32F);
        Calib3d.Rodrigues(_rmat, _rvec);

        // theta = arccos((trR - 1) / 2)
        // ux = (r32 - r23) / (2 sin theta)
        // uy = ...  uz = ...
        // theta_x = ux * theta
        // theta_y = uy * theta
        // theta_z = uz * theta

        return _rvec;
    }

    // THE OUTPUTS: ----
    public double theta;
    public Mat coordsH = new Mat(4, 1, CvType.CV_32F);
    // ----------------- + tagId which was defined earlier

    public static final Mat emptyMat = new Mat();
    @Override
    public Mat processFrame(Mat input) {
        if(input.empty()){
            return emptyMat;
        }

        if(start && ! kill) {
            int ok = detectAllTags(input);

            if (ok == 0) {
                return input;
            }

            computeTvecRvec(crazyMat);

            Mat transform1 = buildTransformationMatrix(rvec, tvec.reshape(1, new int[]{1, 3}));
            Mat transform = transform1.inv().matMul(transformCamera2Robot);
            coordsH = transform.matMul(originH);

            theta = getRvecFromRmat(transform.submat(0, 3, 0, 3)).get(1, 0)[0] * 180 / Math.PI;

            for (Point p : crazyMat.toArray()) {
                Imgproc.circle(input, p, 5, new Scalar(100, 100, 100));
            }
        }
        return input;
    }

    public Mat getXYZ(){
        return coordsH;
    }

    public double getTheta(){
        return theta;
    }

    public int getTagId(){
        return tagId;
    }
}
