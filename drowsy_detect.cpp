#include<vector>
#include<math.h>

#include <iostream>
#include <dlib/opencv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

#include <time.h>

using namespace dlib;
using namespace std;
using namespace cv;



//Intrisics can be calculated using opencv sample code under opencv/sources/samples/cpp/tutorial_code/calib3d
//Normally, you can also apprximate fx and fy by image width, cx by half image width, cy by half image height instead
double K[9] = { 6.5308391993466671e+002, 0.0, 3.1950000000000000e+002, 0.0, 6.5308391993466671e+002, 2.3950000000000000e+002, 0.0, 0.0, 1.0 };
double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };

//declare side angle
char* side = NULL;
//declare side border

int number;
//double* face_width;
//int* pos_x;
//int* pos_y;

double face_width[8];
int pos_x[8];
int pos_y[8];


//declare variable ratio 
double ratio1, ratio2, ratio3 = NULL;
//declare a vector/stack to store all ratios and push the values for the certain time 
std::vector<double> ratios;

//declare the total concecutive seconds of left eye, right eye, lips, and head tilt down
double counter_seconds[4] = { 0,0,0,0 };
double threshold_value[4] = { 30,30,6,15 }; //threshold value for left eye, right eye, lips and head tilt respectively 
int flag[4] = { 0,0,0,0 }; //to indicate each sign of drowsniness




int main()
{
	//

	side = "center";
	//open webcam
	cv::VideoCapture cap(0);
	
	if (!cap.isOpened())
	{
		cerr << "Unable to connect to camera" << endl;
		return EXIT_FAILURE;
	}

	//net_type net;

	//Load face detection and pose estimation models (dlib).
	frontal_face_detector detector = get_frontal_face_detector();
	shape_predictor predictor;
	deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;
	

	//fill in cam intrinsics and distortion coefficients
	cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
	cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

	//fill in 3D ref points(world coordinates)
	std::vector<cv::Point3d> object_pts;
	object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
	object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
	object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
	object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
	object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
	object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
	object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
	object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
	object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
	object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
	object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
	object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
	object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
	object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner

																		 //2D ref points(image coordinates), referenced from detected facial feature
	std::vector<cv::Point2d> image_pts;

	//result
	cv::Mat rotation_vec;                           //3 x 1
	cv::Mat rotation_mat;                           //3 x 3 R
	cv::Mat translation_vec;                        //3 x 1 T
	cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
	cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

	//reproject 3D points world coordinate axis to verify result pose
	std::vector<cv::Point3d> reprojectsrc;
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 10.0));

	//reprojected 2D points
	std::vector<cv::Point2d> reprojectdst;
	reprojectdst.resize(8);

	//temp buf for decomposeProjectionMatrix()
	cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);



	//text on screen
	ostringstream outtext;

	//DO personal calibration here first if needed (????)

	//main loop
	while (1)
	{

		clock_t tstart = clock();
		// Grab a frame
		cv::Mat temp;
		cap >> temp;
		//resize(temp, temp, Size(640, 480));
		//cv_image<bgr_pixel> cimg;
		// get either left or right side of the frame to be proccessed

		cv::Mat temp1;
		if (strcmp(side, "right") == 0)
			temp1 = temp(Range::all(), Range(0, 319));
		else if (strcmp(side, "left") == 0)
			temp1 = temp(Range::all(), Range(320, 639));
		else if (strcmp(side, "center") == 0)
			temp1 = temp(Range::all(), Range::all());

		cv_image<bgr_pixel> cimg(temp1);

		// Detect faces 
		std::vector<dlib::rectangle> faces = detector(cimg);

		//print how many people captured
		outtext << "Number of people: " << faces.size();
		cv::putText(temp, outtext.str(), cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255));
		outtext.str("");

		number = faces.size();
		//face_width = new double(number);
		//pos_x = new int[number];
		//pos_y = new int[number];

		// Find the pose of each face 
		//only for 1 face(if more, need to loop!!)
		if (faces.size() > 0)
		{
			for (int a = 0; a < faces.size(); a++)
			{
				//track features
				full_object_detection shape = predictor(cimg, faces[a]);

				//draw features
				for (unsigned int i = 0; i < 68; ++i)
				{
					circle(temp1, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
				}

				//fill in 2D ref points
				image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
				image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
				image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
				image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
				image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
				image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
				image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
				image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
				image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
				image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
				image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
				image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
				image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
				image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner


																						  //store each face size and position coordinate
				face_width[a] = shape.part(16).x() - shape.part(0).x();
				pos_x[a] = shape.part(42).x();
				pos_y[a] = shape.part(8).y() + 30;

				//calculate ratio of points
				ratio1 = (shape.part(36) - shape.part(39)).length_squared();
				ratio1 /= (shape.part(37) - shape.part(41)).length_squared();


				//left eye
				ratio2 = (shape.part(42) - shape.part(45)).length_squared();
				ratio2 /= (shape.part(43) - shape.part(47)).length_squared();

				//mouth ratio
				ratio3 = (shape.part(60) - shape.part(64)).length_squared();
				ratio3 /= (shape.part(62) - shape.part(66)).length_squared();

				ratio3 = (shape.part(60) - shape.part(64)).length_squared();
				ratio3 /= (shape.part(62) - shape.part(66)).length_squared();

				//push all values to stack
				ratios.clear();
				ratios.push_back(ratio1);
				ratios.push_back(ratio2);
				ratios.push_back(ratio3);


				//display ratio to cv frame
				outtext << "EYE-MOUTH RATIO ";
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 80), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(400, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");
				outtext << "Left_eye: " << setprecision(3) << ratios[0];
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 90), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(400, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");
				outtext << "Right_eye: " << setprecision(3) << ratios[1];
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 100), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(400, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");
				outtext << "Inner_lips: " << setprecision(3) << ratios[2];
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 110), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(400, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");

				//calc pose
				cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);

				//reproject
				cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);

				//draw axis
				cv::line(temp1, reprojectdst[0], reprojectdst[1], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[1], reprojectdst[2], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[2], reprojectdst[3], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[3], reprojectdst[0], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[4], reprojectdst[5], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[5], reprojectdst[6], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[6], reprojectdst[7], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[7], reprojectdst[4], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[0], reprojectdst[4], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[1], reprojectdst[5], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[2], reprojectdst[6], cv::Scalar(0, 0, 255));
				cv::line(temp1, reprojectdst[3], reprojectdst[7], cv::Scalar(0, 0, 255));

				//calc euler angle
				cv::Rodrigues(rotation_vec, rotation_mat);
				cv::hconcat(rotation_mat, translation_vec, pose_mat);
				cv::decomposeProjectionMatrix(pose_mat, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angle);

				//show angle result on cv frame     
				outtext << "HEAD TRACKING ";
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 30), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");
				outtext << "X: " << setprecision(3) << euler_angle.at<double>(0);
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 40), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");
				outtext << "Y: " << setprecision(3) << euler_angle.at<double>(1);
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 50), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");
				outtext << "Z: " << setprecision(3) << euler_angle.at<double>(2);
				cv::putText(temp, outtext.str(), cv::Point(shape.part(0).x(), shape.part(8).y() + 60), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));
				//cv::putText(temp, outtext.str(), cv::Point(50, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255));
				outtext.str("");

				image_pts.clear();
			}
		}

		//determine which 1 is driver
		double max = 0;
		int index = -1;

		for (int i = 0; i < faces.size(); i++)
		{
			if (face_width[i] > max)
			{
				max = face_width[i];
				index = i;
			}
		}


		for (int i = 0; i < faces.size(); i++)
		{
			if (i == index)
			{
				outtext << "driver ";
				cv::putText(temp, outtext.str(), cv::Point(pos_x[i], pos_y[i]), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
				outtext.str("");
			}
			else
			{
				outtext << "passenger ";
				cv::putText(temp, outtext.str(), cv::Point(pos_x[i], pos_y[i]), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
				outtext.str("");
			}
		}


		//draw rectangle border according to side-angle argument
		if (strcmp(side, "right") == 0)
			cv::rectangle(temp, cv::Point(0, 0), cv::Point(319, 479), cv::Scalar(255, 0, 0));
		else if (strcmp(side, "left") == 0)
			cv::rectangle(temp, cv::Point(320, 0), cv::Point(639, 479), cv::Scalar(255, 0, 0));
		else if (strcmp(side, "center") == 0)
			cv::rectangle(temp, cv::Point(0, 0), cv::Point(639, 479), cv::Scalar(255, 0, 0));

		//press esc to break
		//hd res
		namedWindow("Demo", WINDOW_NORMAL);
		resizeWindow("Demo", 1080, 810);
		
		imshow("Demo", temp);
		
		if (cv::waitKey(1) == 27)
		{
			break;
		}

		//rule based decision
		//capturing timeframe when threshold is passed (blom mbedain mana driver di part ini!)
		if (ratio3!=NULL && ratio3 < threshold_value[2]) {
		//means mounth is in open-wide state; starts the counting seconds
		counter_seconds[2] += (double)(clock() - tstart) / CLOCKS_PER_SEC;
		}
		else {
		//reset the counter seconds during normal state
		counter_seconds[2] = 0;
		flag[2] = 0;
		}

		//decision making
		if (counter_seconds[2] >= 2) {
		flag[2] = 1;
		cout << "Yawning, time elapsed: " << counter_seconds[2] << "  ";
		}

		//decision summary
		if ( (flag[0]==1 && flag[1]==1) || flag[2] == 1 || flag[3]==1) {
		cout << "(Drowsiness detected)" << endl;
		}
		//delete[] face_width;
		//delete[] pos_x;
		//delete[] pos_y;

	}

	return 0;
}



