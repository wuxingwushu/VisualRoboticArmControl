#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "rtsp_demo.h"
#include "sample_comm.h"
#include "yolov5.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Button.hpp"
#include "PWM.hpp"
#include "OLED.hpp"
#include "ADC.hpp"
#include "MenuEvents.hpp"
#include "ArmConsole.hpp"
#include "CameraRays.hpp"

// disp size
int width = 1920;
int height = 1080;

// model size
int model_width = 640;
int model_height = 640;
float scale;
int leftPadding;
int topPadding;

static RK_S32 test_venc_init(int chnId, int width, int height, RK_CODEC_ID_E enType)
{
	printf("%s\n", __func__);
	VENC_RECV_PIC_PARAM_S stRecvParam;
	VENC_CHN_ATTR_S stAttr;
	memset(&stAttr, 0, sizeof(VENC_CHN_ATTR_S));

	// RTSP H264
	stAttr.stVencAttr.enType = enType;
	// stAttr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	stAttr.stVencAttr.enPixelFormat = RK_FMT_RGB888;
	stAttr.stVencAttr.u32Profile = H264E_PROFILE_MAIN;
	stAttr.stVencAttr.u32PicWidth = width;
	stAttr.stVencAttr.u32PicHeight = height;
	stAttr.stVencAttr.u32VirWidth = width;
	stAttr.stVencAttr.u32VirHeight = height;
	stAttr.stVencAttr.u32StreamBufCnt = 2;
	stAttr.stVencAttr.u32BufSize = width * height * 3 / 2;
	stAttr.stVencAttr.enMirror = MIRROR_NONE;

	stAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
	stAttr.stRcAttr.stH264Cbr.u32BitRate = 3 * 1024;
	stAttr.stRcAttr.stH264Cbr.u32Gop = 1;
	RK_MPI_VENC_CreateChn(chnId, &stAttr);

	memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	stRecvParam.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(chnId, &stRecvParam);

	return 0;
}

// demo板dev默认都是0，根据不同的channel 来选择不同的vi节点
int vi_dev_init()
{
	printf("%s\n", __func__);
	int ret = 0;
	int devId = 0;
	int pipeId = devId;

	VI_DEV_ATTR_S stDevAttr;
	VI_DEV_BIND_PIPE_S stBindPipe;
	memset(&stDevAttr, 0, sizeof(stDevAttr));
	memset(&stBindPipe, 0, sizeof(stBindPipe));
	// 0. get dev config status
	ret = RK_MPI_VI_GetDevAttr(devId, &stDevAttr);
	if (ret == RK_ERR_VI_NOT_CONFIG)
	{
		// 0-1.config dev
		ret = RK_MPI_VI_SetDevAttr(devId, &stDevAttr);
		if (ret != RK_SUCCESS)
		{
			printf("RK_MPI_VI_SetDevAttr %x\n", ret);
			return -1;
		}
	}
	else
	{
		printf("RK_MPI_VI_SetDevAttr already\n");
	}
	// 1.get dev enable status
	ret = RK_MPI_VI_GetDevIsEnable(devId);
	if (ret != RK_SUCCESS)
	{
		// 1-2.enable dev
		ret = RK_MPI_VI_EnableDev(devId);
		if (ret != RK_SUCCESS)
		{
			printf("RK_MPI_VI_EnableDev %x\n", ret);
			return -1;
		}
		// 1-3.bind dev/pipe
		stBindPipe.u32Num = pipeId;
		stBindPipe.PipeId[0] = pipeId;
		ret = RK_MPI_VI_SetDevBindPipe(devId, &stBindPipe);
		if (ret != RK_SUCCESS)
		{
			printf("RK_MPI_VI_SetDevBindPipe %x\n", ret);
			return -1;
		}
	}
	else
	{
		printf("RK_MPI_VI_EnableDev already\n");
	}

	return 0;
}

int vi_chn_init(int channelId, int width, int height)
{
	int ret;
	int buf_cnt = 2;
	// VI init
	VI_CHN_ATTR_S vi_chn_attr;
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	vi_chn_attr.stIspOpt.u32BufCount = buf_cnt;
	vi_chn_attr.stIspOpt.enMemoryType =
		VI_V4L2_MEMORY_TYPE_DMABUF; // VI_V4L2_MEMORY_TYPE_MMAP;
	vi_chn_attr.stSize.u32Width = width;
	vi_chn_attr.stSize.u32Height = height;
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE; // COMPRESS_AFBC_16x16;
	vi_chn_attr.u32Depth = 2;
	ret = RK_MPI_VI_SetChnAttr(0, channelId, &vi_chn_attr);
	ret |= RK_MPI_VI_EnableChn(0, channelId);
	if (ret)
	{
		printf("ERROR: create VI error! ret=%d\n", ret);
		return ret;
	}

	return ret;
}

int test_vpss_init(int VpssChn, int width, int height)
{
	printf("%s\n", __func__);
	int s32Ret;
	VPSS_CHN_ATTR_S stVpssChnAttr;
	VPSS_GRP_ATTR_S stGrpVpssAttr;

	int s32Grp = 0;

	stGrpVpssAttr.u32MaxW = 4096;
	stGrpVpssAttr.u32MaxH = 4096;
	stGrpVpssAttr.enPixelFormat = RK_FMT_YUV420SP;
	stGrpVpssAttr.stFrameRate.s32SrcFrameRate = -1;
	stGrpVpssAttr.stFrameRate.s32DstFrameRate = -1;
	stGrpVpssAttr.enCompressMode = COMPRESS_MODE_NONE;

	stVpssChnAttr.enChnMode = VPSS_CHN_MODE_USER;
	stVpssChnAttr.enDynamicRange = DYNAMIC_RANGE_SDR8;
	stVpssChnAttr.enPixelFormat = RK_FMT_RGB888;
	stVpssChnAttr.stFrameRate.s32SrcFrameRate = -1;
	stVpssChnAttr.stFrameRate.s32DstFrameRate = -1;
	stVpssChnAttr.u32Width = width;
	stVpssChnAttr.u32Height = height;
	stVpssChnAttr.enCompressMode = COMPRESS_MODE_NONE;

	s32Ret = RK_MPI_VPSS_CreateGrp(s32Grp, &stGrpVpssAttr);
	if (s32Ret != RK_SUCCESS)
	{
		return s32Ret;
	}

	s32Ret = RK_MPI_VPSS_SetChnAttr(s32Grp, VpssChn, &stVpssChnAttr);
	if (s32Ret != RK_SUCCESS)
	{
		return s32Ret;
	}
	s32Ret = RK_MPI_VPSS_EnableChn(s32Grp, VpssChn);
	if (s32Ret != RK_SUCCESS)
	{
		return s32Ret;
	}

	s32Ret = RK_MPI_VPSS_StartGrp(s32Grp);
	if (s32Ret != RK_SUCCESS)
	{
		return s32Ret;
	}
	return s32Ret;
}

cv::Mat letterbox(cv::Mat input)
{
	float scaleX = (float)model_width / (float)width;	// 0.888
	float scaleY = (float)model_height / (float)height; // 1.125
	scale = scaleX < scaleY ? scaleX : scaleY;

	int inputWidth = (int)((float)width * scale);
	int inputHeight = (int)((float)height * scale);

	leftPadding = (model_width - inputWidth) / 2;
	topPadding = (model_height - inputHeight) / 2;

	cv::Mat inputScale;
	cv::resize(input, inputScale, cv::Size(inputWidth, inputHeight), 0, 0, cv::INTER_LINEAR);
	cv::Mat letterboxImage(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Rect roi(leftPadding, topPadding, inputWidth, inputHeight);
	inputScale.copyTo(letterboxImage(roi));

	return letterboxImage;
}

void mapCoordinates(int *x, int *y)
{
	int mx = *x - leftPadding;
	int my = *y - topPadding;

	*x = (int)((float)mx / scale);
	*y = (int)((float)my / scale);
}

float DifferencePercentage(int a, int b)
{
	if (a > b)
	{
		return float(a - b) / a;
	}
	else
	{
		return float(b - a) / b;
	}
}

bool isRectangle(std::vector<cv::Point> &Spot)
{
	// 坐标点按照顺时针排列
	cv::Point Core = {Spot[0].x + Spot[1].x + Spot[2].x + Spot[3].x, Spot[0].y + Spot[1].y + Spot[2].y + Spot[3].y};
	Core.x /= 4;
	Core.y /= 4;

	std::vector<cv::Point> Angle(4);
	Angle[0] = {Spot[0].x - Core.x, Spot[0].y - Core.y};
	Angle[1] = {Spot[1].x - Core.x, Spot[1].y - Core.y};
	Angle[2] = {Spot[2].x - Core.x, Spot[2].y - Core.y};
	Angle[3] = {Spot[3].x - Core.x, Spot[3].y - Core.y};

	for (size_t i = 0; i < Angle.size(); i++)
	{
		Angle[i].x = atan2(Angle[i].x, Angle[i].y);
	}
	int min = 0, max = 0;
	for (size_t i = 0; i < Angle.size() - 1; i++)
	{
		if (Angle[min].x > Angle[i + 1].x)
		{
			min = i + 1;
		}
		if (Angle[max].x < Angle[i + 1].x)
		{
			max = i + 1;
		}
	}
	if (min != 0)
	{
		std::swap(Angle[min].x, Angle[0].x);
		std::swap(Spot[min], Spot[0]);
	}
	if (max != (Angle.size() - 1))
	{
		std::swap(Angle[max].x, Angle[Angle.size() - 1].x);
		std::swap(Spot[max], Spot[Angle.size() - 1]);
	}
	if (Angle[1].x > Angle[2].x)
	{
		std::swap(Spot[1], Spot[2]);
	}

	const float PCpix = 0.1f;

	// 判断四个点是否构成矩形
	int d1 = (((Spot[0].x - Spot[1].x) * (Spot[0].x - Spot[1].x)) + ((Spot[0].y - Spot[1].y) * (Spot[0].y - Spot[1].y)));
	int d3 = (((Spot[2].x - Spot[3].x) * (Spot[2].x - Spot[3].x)) + ((Spot[2].y - Spot[3].y) * (Spot[2].y - Spot[3].y)));
	if (DifferencePercentage(d1, d3) > PCpix)
		return false;

	int d2 = (((Spot[1].x - Spot[2].x) * (Spot[1].x - Spot[2].x)) + ((Spot[1].y - Spot[2].y) * (Spot[1].y - Spot[2].y)));
	int d4 = (((Spot[3].x - Spot[0].x) * (Spot[3].x - Spot[0].x)) + ((Spot[3].y - Spot[0].y) * (Spot[3].y - Spot[0].y)));
	if (DifferencePercentage(d2, d4) > PCpix)
		return false;

	int diagonal1 = (((Spot[0].x - Spot[2].x) * (Spot[0].x - Spot[2].x)) + ((Spot[0].y - Spot[2].y) * (Spot[0].y - Spot[2].y)));
	int diagonal2 = (((Spot[1].x - Spot[3].x) * (Spot[1].x - Spot[3].x)) + ((Spot[1].y - Spot[3].y) * (Spot[1].y - Spot[3].y)));
	return (DifferencePercentage(diagonal1, diagonal2) > PCpix);
}

void MES(OLED &mOLED, MenuEvents *M, unsigned int x, unsigned int y, bool mode = true)
{
	switch (M->Modes())
	{
	case MenuTxet:
		mOLED.string_UTF8(M->Name(), x, y, mode);
		break;
	case MenuVariables:
		mOLED.string_UTF8(M->Show(), x, y, mode);
		break;
	case MenuSubmenu:
		mOLED.string_UTF8(M->Name(), x, y, mode);
		break;
	case MenuFunction:
		mOLED.string_UTF8(M->Name(), x, y, mode);
		break;

	default:
		mOLED.string_UTF8(M->Name(), x, y, mode);
		break;
	}
}

MenuEvents **GetNewMenu(std::vector<MenuEvents *> &MenuS)
{
	printf("\nNew : %d\n", MenuS.size());
	if (MenuS.size() == 0)
		return nullptr;
	MenuEvents **NewMenuS = new MenuEvents *[MenuS.size()];
	for (size_t i = 0; i < MenuS.size(); ++i)
	{
		NewMenuS[i] = MenuS[i];
	}
	return NewMenuS;
}

std::vector<std::vector<cv::Point>> Hongh(cv::Mat &img)
{
	cv::Mat gray, thresh;
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);		 // 转换为灰度图
	threshold(gray, thresh, 75, 255, cv::THRESH_BINARY); // 转换为二值图像( 0 / 255 )

	std::vector<std::vector<cv::Point>> contours{};
	findContours(thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	std::vector<std::vector<cv::Point>> rectangles{};
	for (const auto &contour : contours)
	{
		double perimeter = arcLength(contour, true);
		std::vector<cv::Point> approx{};
		approxPolyDP(contour, approx, 0.02 * perimeter, true);
		if (approx.size() == 4)
		{
			bool BOXTF = true;
			for (size_t i = 0; i < approx.size(); ++i)
			{
				for (size_t ix = i + 1; ix < approx.size(); ++ix)
				{
					int JL = ((approx[i].x - approx[ix].x) * (approx[i].x - approx[ix].x) + (approx[i].y - approx[ix].y) * (approx[i].y - approx[ix].y));
					if (JL < 30 * 30)
					{
						BOXTF = false;
						goto ONBox;
					}
				}
			}
		ONBox:
			if (BOXTF)
			{
				rectangles.push_back(approx);
			}
		}
	}
	return rectangles;
}
#include <iostream>
#include <fstream>
void saveAsPPM(const cv::Mat &image, const std::string &filename)
{
	// 打开文件
	std::ofstream file(filename, std::ios::out | std::ios::binary);
	if (!file.is_open())
	{
		std::cerr << "Error: Couldn't open file for writing." << std::endl;
		return;
	}

	// 写入PPM文件头
	file << "P6\n"
		 << image.cols << " " << image.rows << "\n255\n";

	// 写入像素数据
	for (int y = 0; y < image.rows; ++y)
	{
		for (int x = 0; x < image.cols; ++x)
		{
			cv::Vec3b pixel = image.at<cv::Vec3b>(y, x);
			file.put(pixel[2]); // R
			file.put(pixel[1]); // G
			file.put(pixel[0]); // B
		}
	}

	// 关闭文件
	file.close();
}

unsigned int PZshu = 0;
RK_S32 s32Ret = 0;
void *data;
void PZFF(void *C, void *D)
{
	if (s32Ret != RK_SUCCESS)
	{
		return;
	}
	cv::Mat frame(height, width, CV_8UC3, data);
	saveAsPPM(frame, ("/mnt/sdcard/" + std::to_string(PZshu) + ".ppm").c_str());
	++PZshu;
}

void PCFF(void *C, void *D)
{
	ArmConsole *FArmConsole = (ArmConsole *)C;
	FArmConsole->Claw(*(int *)D);
}

double GetBianA(double A, double B, double AB)
{
	double C_rad = M_PI - B - A;
	if (sin(C_rad) == 0)
		return 0;
	return AB * sin(A) / sin(C_rad);
}

double GetBianB(double A, double B, double AB)
{
	double C_rad = M_PI - B - A;
	if (sin(C_rad) == 0)
		return 0;
	return AB * sin(B) / sin(C_rad);
}


CameraRays mCameraRays(3.95, 5.59, 3.14, 2304 * 1.5f, 1296 * 1.8f);

Arm_State State = {0,7,7,0,OuterTriangle};

ArmConsole mArmConsole(State);



// Rknn mode
rknn_app_context_t rknn_app_ctx;
object_detect_result_list od_results;
int ret;
Arm_State ZCState;
float probability;
int BoxCoreX = 0, BoxCoreY = 0;
char text[16];
int sX, sY, eX, eY;
int ZFXsX, ZFXsY, ZFXeX, ZFXeY;
// h264_frame
VENC_STREAM_S stFrame;
VIDEO_FRAME_INFO_S stVpssFrame;

float PXangle = 0.0;
bool WTbool;
double CWAngle, CHAngle;
void YOLOV5()
{
	// vpss 获取摄像头帧数据
	s32Ret = RK_MPI_VPSS_GetChnFrame(0, 0, &stVpssFrame, -1);
	if (s32Ret == RK_SUCCESS)
	{
		data = RK_MPI_MB_Handle2VirAddr(stVpssFrame.stVFrame.pMbBlk);
		// opencv
		cv::Mat frame(height, width, CV_8UC3, data);
		cv::Mat letterboxImage = letterbox(frame);
		memcpy(rknn_app_ctx.input_mems[0]->virt_addr, letterboxImage.data, model_width * model_height * 3);
		inference_yolov5_model(&rknn_app_ctx, &od_results);
		for (int i = 0; i < od_results.count; ++i)
		{
			// 获取框的四个坐标
			if (od_results.count >= 1)
			{
				object_detect_result *det_result = &(od_results.results[i]);
				/*printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
					 det_result->box.left, det_result->box.top,
					 det_result->box.right, det_result->box.bottom,
					 det_result->prop);*/

				sX = (int)(det_result->box.left);
				sY = (int)(det_result->box.top);
				eX = (int)(det_result->box.right);
				eY = (int)(det_result->box.bottom);
				mapCoordinates(&sX, &sY);
				mapCoordinates(&eX, &eY);
				if (det_result->prop > probability)
				{
					WTbool = true;
					probability = det_result->prop;
					ZFXsX = sX;
					ZFXsY = sY;
					ZFXeX = eX;
					ZFXeY = eY;
					BoxCoreX = (sX + eX) / 2;
					BoxCoreY = (sY + eY) / 2;
				}

				cv::rectangle(frame, cv::Point(sX, sY),
							  cv::Point(eX, eY),
							  cv::Scalar(0, 255, 0), 3);
				memset(text, 0, 8); // 前 8 个字符全部设置为 0
				sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
				cv::putText(frame, text, cv::Point(sX, sY - 8),
							cv::FONT_HERSHEY_SIMPLEX, 1,
							cv::Scalar(0, 255, 0), 2);
			}
		}

		if (WTbool)
		{
			if (ZFXsX > ZFXeX)
			{
				int hxkkk = ZFXeX;
				ZFXeX = ZFXsX;
				ZFXsX = hxkkk;
			}
			if (ZFXsY > ZFXeY)
			{
				int hxkkk = ZFXeY;
				ZFXeY = ZFXsY;
				ZFXsY = hxkkk;
			}
			ZFXsX -= 10;
			if (ZFXsX < 0)
				ZFXsX = 0;
			ZFXsY -= 10;
			if (ZFXsY < 0)
				ZFXsY = 0;
			ZFXeX += 10;
			if (ZFXeX >= width)
				ZFXeX = width - 1;
			ZFXeY += 10;
			if (ZFXeY >= height)
				ZFXeY = height - 1;

			cv::Rect roi(ZFXsX, ZFXsY, ZFXeX - ZFXsX, ZFXeY - ZFXsY);
			cv::Mat croppedImage = frame(roi);
			std::vector<std::vector<cv::Point>> rectangles = Hongh(croppedImage);
			// if(rectangles.size() != 0)drawContours(frame, rectangles, -1, cv::Scalar(0, 255, 0), 2);

			if (rectangles.size() > 0)
			{
				int gao = 2000;
				int gaoID = 0;
				if (rectangles.size() > 1)
				{
					for (size_t i = 0; i < rectangles.size(); i++)
					{
						int g = 0;
						for (size_t j = 0; j < rectangles[i].size(); j++)
						{
							g += rectangles[i][j].y;
						}
						g /= rectangles[i].size();
						if (gao > g)
						{
							gao = g;
							gaoID = i;
						}
					}
				}
				BoxCoreX = 0;
				BoxCoreY = 0;
				for (size_t j = 0; j < rectangles[gaoID].size(); j++)
				{
					BoxCoreY += rectangles[gaoID][j].y;
					BoxCoreX += rectangles[gaoID][j].x;
				}
				BoxCoreY /= rectangles[gaoID].size();
				BoxCoreX /= rectangles[gaoID].size();

				BoxCoreX += ZFXsX;
				BoxCoreY += ZFXsY;
				cv::circle(frame, {BoxCoreX, BoxCoreY}, 2, cv::Scalar(0, 255, 0), cv::FILLED);
			}

			const float MuBiaoZ = -3.5;
			Arm_Result Result = mArmConsole.GetResult();
			State = mArmConsole.GetState();
			CHAngle = mCameraRays.HeighAngle(height, BoxCoreY) + PXangle;
			CWAngle = mCameraRays.WidthAngle(width, BoxCoreX);
			printf("CameraRays: %f, %f\n", CHAngle / M_PI * 180, CWAngle / M_PI * 180);
			double AngleX = (M_PI / 2) - Result.AngleOAB - Result.AngleABC - Result.AngleBCD;
			double AngleZ = -((M_PI / 2) - Result.AngleTerritory);
			printf("CameraAngle: %f, %f\n", AngleX / M_PI * 180, AngleZ / M_PI * 180);
			double Nx = 0, Ny = 0, Nz = 1;
			rotateX(AngleX + CHAngle, Ny, Nz);
			rotateZ(AngleZ - CWAngle, Nx, Ny);
			double Sx = 1, Sy = -1.5, Sz = -5;
			rotateX(AngleX, Sy, Sz);
			rotateZ(AngleZ, Sx, Sy);
			printf("CP: %f, %f, %f\n", Sx, Sy, Sz);

			double NGao = fabs(MuBiaoZ - State.z - Sz);
			double BAngleX = -(AngleX + CHAngle) - M_PI;
			Ny = GetBianA(BAngleX, M_PI / 2, NGao);
			double BHX = GetBianB(BAngleX, M_PI / 2, NGao);
			Nx = GetBianA(CWAngle, M_PI / 2, BHX);
			rotateZ(AngleZ, Nx, Ny);

			/*double NGao = fabs(MuBiaoZ - State.z - Sz);
			Nx = NGao / Nz * Nx;
			Ny = NGao / Nz * Ny;
			double BFB = (Ny * Ny) / (NGao * NGao) + 1;
			printf("BFB: %f\n", BFB);
			Nx = BFB * Nx;
			Nz = MuBiaoZ;*/
			printf("ZFT: %f, %f, %f\n", Nx, Ny, Nz);
			Nx += State.x + Sx;
			Ny += State.y + Sy;
			printf("MB: %f, %f, %f\n\n", Nx, Ny, Nz);

			ZCState.x = Nx;
			ZCState.y = Ny;
			ZCState.z = -3;
		}
	}
	// 释放补抓画面
	s32Ret = RK_MPI_VPSS_ReleaseChnFrame(0, 0, &stVpssFrame);
	if (s32Ret != RK_SUCCESS)
	{
		RK_LOGE("RK_MPI_VI_ReleaseChnFrame fail %x", s32Ret);
	}
}

void ArmFunction(void *Class, void *Data)
{
	ArmConsole *FArmConsole = (ArmConsole *)Class;
	Arm_State *FData = (Arm_State *)Data;
	printf("X:%f\n", FData->x);
	printf("Y:%f\n", FData->y);
	printf("Z:%f\n", FData->z);
	if (FArmConsole->ArmActionFormula(*FData))
		printf("OK!\n");
	else
		printf("NO!\n");
	
	std::chrono::milliseconds delay2(1000);
    std::this_thread::sleep_for(delay2);
	FArmConsole->Claw(1200000);
};

void ArmFY(void *Class, void *Data)
{
	ArmConsole *FArmConsole = (ArmConsole *)Class;
	Arm_State *FData = (Arm_State *)Data;
	FData->x = 0;
	FData->y = 7;
	FData->z = 7;
	if (FArmConsole->ArmActionFormula(*FData))
		printf("OK!\n");
	else
		printf("NO!\n");

	std::chrono::milliseconds delay2(1000);
    std::this_thread::sleep_for(delay2);
	FArmConsole->Claw(700000);
};

void FlowFunction(void *Class, void *Data)
{
	ArmConsole *FArmConsole = (ArmConsole *)Class;
	Arm_State myState = FArmConsole->GetState();
	float L = 7;
	float P = M_PI / 180 * -70;
	Arm_State *FData = (Arm_State *)Data;
	FData->x = 0;
	FData->y = 7;
	FData->z = 7;
	FArmConsole->ArmActionFormula(*FData);
	bool tc = false;

	std::chrono::milliseconds delay(300);
    


	while (1)
	{
		printf("观测：%f ---\n", P / M_PI * 180);
		P += M_PI / 180 * 5;
		if (P > M_PI / 180 * 70)
			break;
		double Nx = 0, Ny = L;
		rotateZ(P, Nx, Ny);
		FData->z = 7;
		FData->x = Nx;
		FData->y = Ny;
		FArmConsole->ArmActionFormula(*FData);
		std::this_thread::sleep_for(delay);

		// vpss 获取摄像头帧数据
		s32Ret = RK_MPI_VPSS_GetChnFrame(0, 0, &stVpssFrame, -1);
		if (s32Ret == RK_SUCCESS)
		{
			// 摄像头帧数据 物理地址映射到虚拟地址
			data = RK_MPI_MB_Handle2VirAddr(stVpssFrame.stVFrame.pMbBlk);
			// opencv
			cv::Mat frame(height, width, CV_8UC3, data);
			cv::Mat letterboxImage = letterbox(frame);
			memcpy(rknn_app_ctx.input_mems[0]->virt_addr, letterboxImage.data, model_width * model_height * 3);
			inference_yolov5_model(&rknn_app_ctx, &od_results);
			if (od_results.count >= 1)
			{
				for (size_t i = 0; i < od_results.count; ++i)
				{
					object_detect_result *det_result = &(od_results.results[i]);
					sX = (int)(det_result->box.left);
					sY = (int)(det_result->box.top);
					eX = (int)(det_result->box.right);
					eY = (int)(det_result->box.bottom);
					mapCoordinates(&sX, &sY);
					mapCoordinates(&eX, &eY);
					sX += eX;
					sX /= 2;
					sX -= width / 2;
					sX = abs(sX);
					if (sX < 200)
						tc = true;
					else
						printf(">200: %d\n", sX);
				}
			}
		}
		else
			tc = true;

		// 释放补抓画面
		s32Ret = RK_MPI_VPSS_ReleaseChnFrame(0, 0, &stVpssFrame);
		if (s32Ret != RK_SUCCESS)
		{
			RK_LOGE("RK_MPI_VI_ReleaseChnFrame fail %x", s32Ret);
		}
		if (tc)
			break;
	};
	State = *FData;
	float Angle = -((M_PI / 2) - FArmConsole->GetResult().AngleTerritory);

	
	while (1)
	{
		printf("微调：%f - %f ---\n", Angle / M_PI * 180, CWAngle / M_PI * 180);
		YOLOV5();
		if (fabs(CWAngle) < (M_PI / 60))
			break;
		Angle += (M_PI / 360) * (CWAngle > 0 ? -1 : 1);
		double Nx = 0, Ny = L;
		rotateZ(Angle, Nx, Ny);
		FData->x = Nx;
		FData->y = Ny;
		FData->z = 7;
		FArmConsole->ArmActionFormula(*FData);
		State = *FData;
    	std::this_thread::sleep_for(delay);
	}

	FArmConsole->ArmActionFormula(*FData);
	std::chrono::milliseconds delay2(1000);
    std::this_thread::sleep_for(delay2);
	FData->x = 0;
	FData->y = 7;
	FData->z = 7;
	FArmConsole->Claw(1200000);
	std::this_thread::sleep_for(delay);
	FArmConsole->ArmActionFormula(*FData);
	std::this_thread::sleep_for(delay2);
	FArmConsole->Claw(700000);
}

int main(int argc, char *argv[])
{
	OLED mOLED(128, 160);
	printf("OLED\n");

	

	int Pca = 0;

	// ADC ADC1(1);
	// printf("\nADC\n");

	Button mGpioW(57, 0);
	Button mGpioS(56, 0);
	Button mGpioA(68, 0);
	Button mGpioD(69, 0);
	printf("Button\n");

	

	bool RtspBool = true;  // 视频推流开关
	bool YoloBool = true;  // Yolo 开关
	bool HoughBool = true; // Hough 开关
	bool YoloSBbool = false;

	std::vector<MenuEvents *> SubmenuS{};
	std::vector<MenuEvents *> MenuS{};

	MenuS.clear();
	SubmenuS.clear();
	MenuS.push_back(new Variables("x", &State.x, Float, 40, 40));
	MenuS.push_back(new Variables("y", &State.y, Float, 0, 40));
	MenuS.push_back(new Variables("z", &State.z, Float, -13, 40));
	MenuS.push_back(new Variables("angle", &State.angle, Float, -90, 90));
	MenuS.push_back(new Variables("mode", &State.mode, Enum, 0, Enum));
	MenuS.push_back(new Function("执行", ArmFY, &mArmConsole, &State));
	SubmenuS.push_back(new Submenu("机械臂状态", GetNewMenu(MenuS), MenuS.size()));

	MenuS.clear();
	MenuS.push_back(new Variables("偏差", &Pca, Int, -1000000, 1000000));		
	MenuS.push_back(new Function("偏差执行", PCFF, &mArmConsole, &Pca));
	MenuS.push_back(new Variables("储存名", &PZshu, uInt, 0, 1000));
	MenuS.push_back(new Function("拍照", PZFF, &data, &data));
	SubmenuS.push_back(new Submenu("拍照模式", GetNewMenu(MenuS), MenuS.size()));

	MenuS.clear();
	MenuS.push_back(new Variables("x", &ZCState.x, Float, -40, 40));
	MenuS.push_back(new Variables("y", &ZCState.y, Float, 0, 40));
	MenuS.push_back(new Variables("预测", &YoloSBbool, Bool, 0, 1, true));
	MenuS.push_back(new Function("抓取", ArmFunction, &mArmConsole, &ZCState));
	MenuS.push_back(new Function("复原", ArmFY, &mArmConsole, &State));
	MenuS.push_back(new Function("测试", FlowFunction, &mArmConsole, &ZCState));
	SubmenuS.push_back(new Submenu("测试位置", GetNewMenu(MenuS), MenuS.size()));

	MenuS.clear();
	MenuS.push_back(new Variables("PX", &PXangle, Float, -40, 40));
	MenuS.push_back(new Variables("视频推流", &RtspBool, Bool, 0, 1, true));
	MenuS.push_back(new Variables("Yolo", &YoloBool, Bool, 0, 1, true));
	MenuS.push_back(new Variables("Hough", &HoughBool, Bool, 0, 1, true));
	SubmenuS.push_back(new Submenu("设置", GetNewMenu(MenuS), MenuS.size()));

	mMenuXX = new Submenu("主页", GetNewMenu(SubmenuS), SubmenuS.size());
	MenuS.clear();
	SubmenuS.clear();
	printf("Menu\n");

	// Rknn mode
	/*rknn_app_context_t rknn_app_ctx;
	object_detect_result_list od_results;
	int ret;*/
	const char *model_path = "./model/yolov5.rknn";
	memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
	init_yolov5_model(model_path, &rknn_app_ctx);
	printf("init rknn model success!\n");
	init_post_process();

	// h264_frame
	VENC_STREAM_S stFrame;
	stFrame.pstPack = (VENC_PACK_S *)malloc(sizeof(VENC_PACK_S));
	VIDEO_FRAME_INFO_S h264_frame;

	// rkaiq init
	RK_BOOL multi_sensor = RK_FALSE;
	const char *iq_dir = "/etc/iqfiles";
	rk_aiq_working_mode_t hdr_mode = RK_AIQ_WORKING_MODE_NORMAL;
	// hdr_mode = RK_AIQ_WORKING_MODE_ISP_HDR2;
	SAMPLE_COMM_ISP_Init(0, hdr_mode, multi_sensor, iq_dir);
	SAMPLE_COMM_ISP_Run(0);

	// rkmpi init
	if (RK_MPI_SYS_Init() != RK_SUCCESS)
	{
		RK_LOGE("rk mpi sys init fail!");
		return -1;
	}

	// rtsp init
	rtsp_demo_handle g_rtsplive = NULL;
	rtsp_session_handle g_rtsp_session;
	g_rtsplive = create_rtsp_demo(554);
	g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/0");
	rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
	rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime());

	// vi init
	vi_dev_init();
	vi_chn_init(0, width, height);

	// vpss init
	test_vpss_init(0, width, height);

	// bind vi to vpss
	MPP_CHN_S stSrcChn, stvpssChn;
	stSrcChn.enModId = RK_ID_VI;
	stSrcChn.s32DevId = 0;
	stSrcChn.s32ChnId = 0;

	stvpssChn.enModId = RK_ID_VPSS;
	stvpssChn.s32DevId = 0;
	stvpssChn.s32ChnId = 0;
	printf("====RK_MPI_SYS_Bind vi0 to vpss0====\n");
	s32Ret = RK_MPI_SYS_Bind(&stSrcChn, &stvpssChn);
	if (s32Ret != RK_SUCCESS)
	{
		RK_LOGE("bind 0 ch venc failed");
		return -1;
	}

	// venc init
	RK_CODEC_ID_E enCodecType = RK_VIDEO_ID_AVC;
	test_venc_init(0, width, height, enCodecType);

	printf("venc init success\n");

	char BanTan[10];
	bool OLEDBool = false;
	bool ZhengGX = true;
	printf("**********************************************************************************************\n");
	while (1)
	{
		probability = 0;
		WTbool = false;
		

		if (mGpioW.Get())
		{
			mMenuXX->W();
			ZhengGX = true;
		}
		if (mGpioS.Get())
		{
			mMenuXX->S();
			ZhengGX = true;
		}
		if (mGpioA.Get())
		{
			mMenuXX->A();
			ZhengGX = true;
		}
		if (mGpioD.Get())
		{
			mMenuXX->D();
			ZhengGX = true;
		}
		if (mGpioS.GetLongPress())
		{
			mMenuXX->LongPress();
			ZhengGX = true;
		}

		if (ZhengGX)
		{
			ZhengGX = false;
			mOLED.LCD_Clear(Color_BLACK, OLEDBool);
			if (mMenuXX->Modes() == MenuSubmenu)
			{
				mOLED.string_UTF8(">", 0, 22 * ((Submenu *)mMenuXX)->ID(), OLEDBool);
				for (size_t i = 0; i < ((Submenu *)mMenuXX)->Max(); ++i)
				{
					MES(mOLED, ((Submenu *)mMenuXX)->GetMenuEvents(i), 20, i * 22, OLEDBool);
				}
			}
			else
			{
				MES(mOLED, mMenuXX, 0, 0, OLEDBool);
				if (mMenuXX->Modes() == MenuVariables)
					mOLED.string_UTF8(std::to_string(((Variables *)mMenuXX)->GetmV()).c_str(), 0, 22, OLEDBool);
			}
			if (!OLEDBool)
			{
				mOLED.ShowFrame();
			}
		}

		if (YoloSBbool)
		{
			YoloSBbool = false;
			ZhengGX = true;
			// vpss 获取摄像头帧数据
			s32Ret = RK_MPI_VPSS_GetChnFrame(0, 0, &stVpssFrame, -1);
			if (s32Ret == RK_SUCCESS)
			{
				data = RK_MPI_MB_Handle2VirAddr(stVpssFrame.stVFrame.pMbBlk);
				// opencv
				cv::Mat frame(height, width, CV_8UC3, data);

				if (YoloBool)
				{
					cv::Mat letterboxImage = letterbox(frame);
					memcpy(rknn_app_ctx.input_mems[0]->virt_addr, letterboxImage.data, model_width * model_height * 3);
					inference_yolov5_model(&rknn_app_ctx, &od_results);
					for (int i = 0; i < od_results.count; i++)
					{
						// 获取框的四个坐标
						if (od_results.count >= 1)
						{
							object_detect_result *det_result = &(od_results.results[i]);
							/*printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
								 det_result->box.left, det_result->box.top,
								 det_result->box.right, det_result->box.bottom,
								 det_result->prop);*/

							sX = (int)(det_result->box.left);
							sY = (int)(det_result->box.top);
							eX = (int)(det_result->box.right);
							eY = (int)(det_result->box.bottom);
							mapCoordinates(&sX, &sY);
							mapCoordinates(&eX, &eY);
							if (det_result->prop > probability)
							{
								WTbool = true;
								probability = det_result->prop;
								ZFXsX = sX;
								ZFXsY = sY;
								ZFXeX = eX;
								ZFXeY = eY;
								BoxCoreX = (sX + eX) / 2;
								BoxCoreY = (sY + eY) / 2;
							}

							cv::rectangle(frame, cv::Point(sX, sY),
										  cv::Point(eX, eY),
										  cv::Scalar(0, 255, 0), 3);
							memset(text, 0, 8); // 前 8 个字符全部设置为 0
							sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
							cv::putText(frame, text, cv::Point(sX, sY - 8),
										cv::FONT_HERSHEY_SIMPLEX, 1,
										cv::Scalar(0, 255, 0), 2);
						}
					}
				}

				if (WTbool && HoughBool)
				{
					if (ZFXsX > ZFXeX)
					{
						int hxkkk = ZFXeX;
						ZFXeX = ZFXsX;
						ZFXsX = hxkkk;
					}
					if (ZFXsY > ZFXeY)
					{
						int hxkkk = ZFXeY;
						ZFXeY = ZFXsY;
						ZFXsY = hxkkk;
					}
					ZFXsX -= 10;
					if (ZFXsX < 0)
						ZFXsX = 0;
					ZFXsY -= 10;
					if (ZFXsY < 0)
						ZFXsY = 0;
					ZFXeX += 10;
					if (ZFXeX >= width)
						ZFXeX = width - 1;
					ZFXeY += 10;
					if (ZFXeY >= height)
						ZFXeY = height - 1;

					cv::Rect roi(ZFXsX, ZFXsY, ZFXeX - ZFXsX, ZFXeY - ZFXsY);
					cv::Mat croppedImage = frame(roi);
					std::vector<std::vector<cv::Point>> rectangles = Hongh(croppedImage);
					// if(rectangles.size() != 0)drawContours(frame, rectangles, -1, cv::Scalar(0, 255, 0), 2);

					if (rectangles.size() > 0)
					{
						int gao = 2000;
						int gaoID = 0;
						if (rectangles.size() > 1)
						{
							for (size_t i = 0; i < rectangles.size(); i++)
							{
								int g = 0;
								for (size_t j = 0; j < rectangles[i].size(); j++)
								{
									g += rectangles[i][j].y;
								}
								g /= rectangles[i].size();
								if (gao > g)
								{
									gao = g;
									gaoID = i;
								}
							}
						}
						BoxCoreX = 0;
						BoxCoreY = 0;
						for (size_t j = 0; j < rectangles[gaoID].size(); j++)
						{
							BoxCoreY += rectangles[gaoID][j].y;
							BoxCoreX += rectangles[gaoID][j].x;
						}
						BoxCoreY /= rectangles[gaoID].size();
						BoxCoreX /= rectangles[gaoID].size();

						BoxCoreX += ZFXsX;
						BoxCoreY += ZFXsY;
						cv::circle(frame, {BoxCoreX, BoxCoreY}, 2, cv::Scalar(0, 255, 0), cv::FILLED);
					}
				}

				if (WTbool)
				{
					const float MuBiaoZ = -3.5;
					Arm_Result Result = mArmConsole.GetResult();
					State = mArmConsole.GetState();
					double HAngle = mCameraRays.HeighAngle(height, BoxCoreY) + PXangle;
					double WAngle = mCameraRays.WidthAngle(width, BoxCoreX);
					printf("CameraRays: %f, %f\n", HAngle / M_PI * 180, WAngle / M_PI * 180);
					double AngleX = (M_PI / 2) - Result.AngleOAB - Result.AngleABC - Result.AngleBCD;
					double AngleZ = -((M_PI / 2) - Result.AngleTerritory);
					printf("CameraAngle: %f, %f\n", AngleX / M_PI * 180, AngleZ / M_PI * 180);
					double Nx = 0, Ny = 0, Nz = 1;
					rotateX(AngleX + HAngle, Ny, Nz);
					rotateZ(AngleZ - WAngle, Nx, Ny);
					double Sx = 1, Sy = -1.5, Sz = -5;
					rotateX(AngleX, Sy, Sz);
					rotateZ(AngleZ, Sx, Sy);
					printf("CP: %f, %f, %f\n", Sx, Sy, Sz);

					double NGao = fabs(MuBiaoZ - State.z - Sz);
					double BAngleX = -(AngleX + HAngle) - M_PI;
					Ny = GetBianA(BAngleX, M_PI / 2, NGao);
					double BHX = GetBianB(BAngleX, M_PI / 2, NGao);
					Nx = GetBianA(WAngle, M_PI / 2, BHX);
					rotateZ(AngleZ, Nx, Ny);

					/*double NGao = fabs(MuBiaoZ - State.z - Sz);
					Nx = NGao / Nz * Nx;
					Ny = NGao / Nz * Ny;
					double BFB = (Ny * Ny) / (NGao * NGao) + 1;
					printf("BFB: %f\n", BFB);
					Nx = BFB * Nx;
					Nz = MuBiaoZ;*/
					printf("ZFT: %f, %f, %f\n", Nx, Ny, Nz);
					Nx += State.x + Sx;
					Ny += State.y + Sy;
					printf("MB: %f, %f, %f\n\n", Nx, Ny, Nz);

					ZCState.x = Nx;
					ZCState.y = Ny;
					ZCState.z = -3;
				}

				if (RtspBool)
					memcpy(data, frame.data, width * height * 3); // 将图片覆盖原图（为后面视频推流处理后的画面）
			}
		}else{
			s32Ret = RK_MPI_VPSS_GetChnFrame(0, 0, &stVpssFrame, -1);
			if (s32Ret == RK_SUCCESS)
			{
				data = RK_MPI_MB_Handle2VirAddr(stVpssFrame.stVFrame.pMbBlk);
			}
		}

		// 推流推流
		if (RtspBool)
		{
			// 发送数据到 VENC 中编码为 H264 格式
			RK_MPI_VENC_SendFrame(0, &stVpssFrame, -1);
			// 进行 rtsp 推流
			s32Ret = RK_MPI_VENC_GetStream(0, &stFrame, -1);
			if (s32Ret == RK_SUCCESS)
			{
				if (g_rtsplive && g_rtsp_session)
				{
					// printf("len = %d PTS = %d \n",stFrame.pstPack->u32Len, stFrame.pstPack->u64PTS);

					void *pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
					rtsp_tx_video(g_rtsp_session, (uint8_t *)pData, stFrame.pstPack->u32Len,
								  stFrame.pstPack->u64PTS);
					rtsp_do_event(g_rtsplive);
				}
			}
		}

		// 释放补抓画面
		s32Ret = RK_MPI_VPSS_ReleaseChnFrame(0, 0, &stVpssFrame);
		if (s32Ret != RK_SUCCESS)
		{
			RK_LOGE("RK_MPI_VI_ReleaseChnFrame fail %x", s32Ret);
		}
		s32Ret = RK_MPI_VENC_ReleaseStream(0, &stFrame);
		if (s32Ret != RK_SUCCESS)
		{
			RK_LOGE("RK_MPI_VENC_ReleaseStream fail %x", s32Ret);
		}
	}

	RK_MPI_SYS_UnBind(&stSrcChn, &stvpssChn);

	RK_MPI_VI_DisableChn(0, 0);
	RK_MPI_VI_DisableDev(0);

	RK_MPI_VPSS_StopGrp(0);
	RK_MPI_VPSS_DestroyGrp(0);

	RK_MPI_VENC_StopRecvFrame(0);
	RK_MPI_VENC_DestroyChn(0);

	free(stFrame.pstPack);

	if (g_rtsplive)
		rtsp_del_demo(g_rtsplive);
	SAMPLE_COMM_ISP_Stop(0);

	RK_MPI_SYS_Exit();
	// Release rknn model
	release_yolov5_model(&rknn_app_ctx);
	deinit_post_process();

	return 0;
}
