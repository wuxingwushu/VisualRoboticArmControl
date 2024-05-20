#ifndef _RKNN_YOLOV5_DEMO_CameraRays_H_
#define _RKNN_YOLOV5_DEMO_CameraRays_H_

#include <complex>

class CameraRays
{
private:
	const double FocalLength;       // 焦距 mm
	const double SensorWidth;       // 传感器宽度 mm
	const double SensorHeigh;       // 传感器高度 mm
	const int StaticWidth;			// 静态宽度
	const int StaticHeigh;			// 静态高度
public:
	CameraRays(const double focalLength, const double sensorWidth, const double sensorHeigh, const unsigned int staticWidth, const unsigned int staticHeigh)
		:FocalLength(focalLength), SensorWidth(sensorWidth), SensorHeigh(sensorHeigh), StaticWidth(staticWidth), StaticHeigh(staticHeigh) {};
	~CameraRays() {};

	float WidthAngle(int Width, int W) {
		double Fw = double(W + ((StaticWidth - Width) / 2) - (StaticWidth / 2)) / (StaticWidth / 2);
		Fw *= SensorWidth;
		double L = sqrt(pow(FocalLength, 2) + pow(Fw, 2));
		double FwAngle = acos((pow(L, 2) + pow(FocalLength, 2) - pow(Fw, 2)) / (2 * L * FocalLength));
		return FwAngle * (Fw < 0 ? -1 : 1);
	}

	float HeighAngle(int Heigh, int H) {
		double Fh = double(H + ((StaticHeigh - Heigh) / 2) - (StaticHeigh / 2)) / (StaticHeigh / 2);
		Fh *= SensorHeigh;
		double L = sqrt(pow(FocalLength, 2) + pow(Fh, 2));
		double FhAngle = acos((pow(L, 2) + pow(FocalLength, 2) - pow(Fh, 2)) / (2 * L * FocalLength));
		return FhAngle * (Fh < 0 ? -1 : 1);
	}
};



#endif
