/* Copyright 18.10.2017 Ikol */
#include "visualizer.hpp"
void test1(int a, vis::Visualizer* visualizer);
void test2(int a, vis::Visualizer* visualizer);
void test3(int a, vis::Visualizer* visualizer);
void test4(int a, vis::Visualizer* visualizer);
// void test5(int a, vis::Visualizer* visualizer);
// void test6(int a, vis::Visualizer* visualizer);
void test7(int a, vis::Visualizer* visualizer);

int main() {
	try {
		vis::Visualizer v(true);

		v.StartVisualizer();

		float x = 0, y = 0, p = 0;

		test1(-1, &v);
		test2(-1, &v);
		test3(-1, &v);
		test4(-1, &v);
		// test5(-1, &v);
		test7(0, &v);  // type 0-int

		int ite = 0;
		int step = 0;
		int a = 0;
		
		IplImage* frame = 0;
		CvCapture* capture = cvCreateFileCapture("video.avi");
		IplImage* image;
		CImage img;image = cvLoadImage("img1.jpg");
		while (v.isRun()) {
			x = x + 0.1;
			p = p + 0.1;
			y = y + 0.1;
				// получаем следующий кадр
				frame = cvQueryFrame(capture);
				img.loadFromIplImage(frame);
				if (!frame) {
					break;
				}
				// показываем кадр
				v.ShowImages(img);

				char c = cvWaitKey(33);
				if (c == 27) { // если нажата ESC - выходим
					break;
				}
			
			ite++;
			if (ite % 10 == 0) {
				v.ClearData(vis::Digitized);
				test1(step, &v);
				v.ClearData(vis::Current);
				test2(step, &v);

				// test6(step, &v);

				test3(step, &v);
				test4(step, &v);
				// test5(step, &v);

				mrpt::poses::CPose2D gpsError;
				gpsError.x() = x;
				gpsError.y() = 0;

				v.SetGpsPos(gpsError, 2);

				step++;

				test7(step, &v);
			}
			if (ite == 50) {
				v.clearTraveledPath();

				test7(step, &v);

				// if set -1 then remove all points
				// if set 0-int then remove all points this type
				v.RemoveErrorPoints(-1);

				ite = 0;
			}

			if (a == 50) {
				// v.RemoveErrorPoints(-1);

				a = 0;
			}
			a++;

			// v.SetCarPos(x, 0, p);

			road_data::ObjPos posCar(
				x,
				0,
				0);

			v.SetCarPos(&posCar);
			/*Sleep(100);*/
			// sleep(0);
				
			
		}
	}
	catch (std::exception &e) {
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...) {
		std::cout << "Untyped exception!!" << std::endl;
		return -1;
	}

	// cin.ignore().get(); //Pause Command for Linux Terminal
	
	return 0;
}

void test1(int a, vis::Visualizer* visualizer) {
	std::list<mrpt::math::TPoint3D> listPoints1 =
		std::list<mrpt::math::TPoint3D>();

	listPoints1.push_back(TPoint3D(a, 1, 0.01));
	listPoints1.push_back(TPoint3D(a + 5, 1, 0.01));
	listPoints1.push_back(TPoint3D(a + 6, 2, 0.01));

	visualizer->AddData(
		vis::Digitized,
		listPoints1,
		road_data::LimitType::Markup,
		road_data::MarkupType::CrossWalk);

	std::list<mrpt::math::TPoint3D> listPoints2 =
		std::list<mrpt::math::TPoint3D>();

	listPoints2.push_back(TPoint3D(a, -1, 0.01));
	listPoints2.push_back(TPoint3D(a + 5, -1, 0.01));
	listPoints2.push_back(TPoint3D(a + 6, 0, 0.01));

	visualizer->AddData(
		vis::Digitized,
		listPoints2,
		road_data::LimitType::Border);
}

void test2(int a, vis::Visualizer* visualizer) {
	std::list<mrpt::math::TPoint3D> listPoints1 =
		std::list<mrpt::math::TPoint3D>();

	listPoints1.push_back(TPoint3D(a, 1.2, 0.01));
	listPoints1.push_back(TPoint3D(a + 5.2, 1, 0.01));
	listPoints1.push_back(TPoint3D(a + 6.2, 2, 0.01));

	visualizer->AddData(
		vis::Current,
		listPoints1,
		road_data::LimitType::Markup,
		road_data::MarkupType::CrossWalk);

	std::list<mrpt::math::TPoint3D> listPoints2 =
		std::list<mrpt::math::TPoint3D>();

	listPoints2.push_back(TPoint3D(a, -1.2, 0.01));
	listPoints2.push_back(TPoint3D(a + 5.2, -1, 0.01));
	listPoints2.push_back(TPoint3D(a + 6.2, 0, 0.01));

	visualizer->AddData(
		vis::Current,
		listPoints2,
		road_data::LimitType::Border);
}

void test3(int a, vis::Visualizer* visualizer) {
	std::list<mrpt::opengl::CBoxPtr> listObjects =
		std::list<mrpt::opengl::CBoxPtr>();

	{
		mrpt::math::TPoint3D p1(a + 5, -2, 2);
		mrpt::math::TPoint3D p2(a + 6, -1, 0);

		mrpt::opengl::CBoxPtr obj = mrpt::opengl::CBox::Create(p1, p2);

		listObjects.push_back(obj);
	}

	{
		mrpt::math::TPoint3D p1(a + 5, 1, 2);
		mrpt::math::TPoint3D p2(a + 6, 2, 0);

		mrpt::opengl::CBoxPtr obj = mrpt::opengl::CBox::Create(p1, p2);

		listObjects.push_back(obj);
	}

	visualizer->AddData(vis::DataType::Digitized,
		listObjects,
		road_data::ObjectType::ObstacleType);
}

void test4(int a, vis::Visualizer* visualizer) {
	std::list<mrpt::opengl::CBoxPtr> listObjects =
		std::list<mrpt::opengl::CBoxPtr>();

	{
		mrpt::math::TPoint3D p1(a + 5, -3, 2);
		mrpt::math::TPoint3D p2(a + 6, -2, 0);

		mrpt::opengl::CBoxPtr obj = mrpt::opengl::CBox::Create(p1, p2);

		listObjects.push_back(obj);
	}

	{
		mrpt::math::TPoint3D p1(a + 5, 2, 2);
		mrpt::math::TPoint3D p2(a + 6, 3, 0);

		mrpt::opengl::CBoxPtr obj = mrpt::opengl::CBox::Create(p1, p2);

		listObjects.push_back(obj);
	}
	

	visualizer->AddData(vis::DataType::Current,
		listObjects,
		road_data::ObjectType::ObstacleType);
}
/*
void test5(int a, vis::Visualizer* visualizer) {
std::list<mrpt::opengl::CBoxPtr> listObjects =
std::list<mrpt::opengl::CBoxPtr>();

{
mrpt::math::TPoint3D p1(a + 7, -3, 0);
mrpt::math::TPoint3D p2(a + 8, -2, 0);

mrpt::opengl::CBoxPtr obj = mrpt::opengl::CBox::Create(p1, p2);

listObjects.push_back(obj);
}

{
mrpt::math::TPoint3D p1(a + 7, 2, 0);
mrpt::math::TPoint3D p2(a + 8, 3, 0);

mrpt::opengl::CBoxPtr obj = mrpt::opengl::CBox::Create(p1, p2);

listObjects.push_back(obj);
}

visualizer->SetPosError(listObjects);
}
*/
/*
void test6(int a, vis::Visualizer* visualizer) {
mrpt::poses::CPose2D point;
point.x() = a;
point.y() = 0;

visualizer->DrawPoints(point, 1, mrpt::utils::TColorf(1, 0, 0));
}
*/
void test7(int a, vis::Visualizer* visualizer) {
	mrpt::poses::CPose2D point;
	point.x() = a + 1;
	point.y() = 0;

	// if set the same type then
	// new point will be add to the same list
	visualizer->DrawErrorPoints(point, 0.5, mrpt::utils::TColorf(0, 0, 1), a);
}
