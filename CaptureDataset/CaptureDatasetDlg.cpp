
// CaptureDatasetDlg.cpp : implementation file
//

#include "stdafx.h"
#include "CaptureDataset.h"
#include "CaptureDatasetDlg.h"
#include "afxdialogex.h"
#include <thread>
#include <math.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

bool pairCompare(const std::pair<double, double>& firstElem, const std::pair<double, double>& secondElem) 
{
	return firstElem.first < secondElem.first;
}

//-----------------------------------------------------------------------------
#define SAVE_PATH          ("D:/Workspace/Dataset/Record_MVCam")
#define LIDAR_COM_BAUDRATE (115200)
#define LIDAR_SPEED        (400)
#define LIDAR_MAP_SIZE     (700)
const size_t LIDAR_RESOLUTION = 360 * 2;
const double LIDAR_RESCALE = (double)LIDAR_MAP_SIZE / 1400.0;
const char *strLidarComPath[] = { "\\\\.\\com3", "\\\\.\\com8" };
//-----------------------------------------------------------------------------

using namespace mvIMPACT::acquire;
using namespace rp::standalone::rplidar;

static volatile int  gNumGrabbing = 0;
static volatile bool gCameraGrabbing[MAX_NUM_CAMERAS];
static cv::Mat       gMatFrames[MAX_NUM_CAMERAS];
static SRWLOCK       gSRWLock[MAX_NUM_CAMERAS];

static volatile int  gNumLidarGrabbing = 0;
static volatile bool gLidarGrabbing[MAX_NUM_LIDARS];
static SRWLOCK       gSRWLidarLock[MAX_NUM_LIDARS];

//-----------------------------------------------------------------------------
inline bool checkRPLIDARHealth(RPlidarDriver * drv)
{
	u_result     op_result;
	rplidar_response_device_health_t healthinfo;

	op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
			// enable the following code if you want rplidar to be reboot by software
			// drv->reset();
			return false;
		}
		else {
			return true;
		}

	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
}


struct LidarParameter
{
	LidarParameter(int _threadID, RPlidarDriver* _p, std::vector<std::pair<double, double>> *_nodes)
		: nThreadID(_threadID)
		, pDriver(_p)
		, nodes(_nodes)
	{}
	~LidarParameter() {}

	int            nThreadID;
	RPlidarDriver* pDriver;
	std::vector<std::pair<double, double>> *nodes; // angle / distance
};
static LidarParameter* stPLidarParams[MAX_NUM_LIDARS];


//-----------------------------------------------------------------------------
unsigned int __stdcall LidarCaptureThread(void* pData)
//-----------------------------------------------------------------------------
{
	LidarParameter* pParam = (LidarParameter*)pData;
	u_result op_result;

	while (gLidarGrabbing[pParam->nThreadID])
	{
		rplidar_response_measurement_node_t nodes[LIDAR_RESOLUTION];
		size_t count = _countof(nodes);
		op_result = pParam->pDriver->grabScanData(nodes, count);

		if (IS_FAIL(op_result)) 
		{
			printf("there is problem to grabbing lidar sensor data\n");
			continue;
		}

		pParam->pDriver->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count; ++pos)
		{
			printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
				(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
				(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
				nodes[pos].distance_q2 / 4.0f,
				nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
		}

		AcquireSRWLockExclusive(&gSRWLidarLock[pParam->nThreadID]);
		for (int pos = 0; pos < LIDAR_RESOLUTION; ++pos)
		{
			if (pos < (int)count)
			{
				(*pParam->nodes)[pos].first = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
				(*pParam->nodes)[pos].second = nodes[pos].distance_q2 / 4.0f;
			}
			else
			{
				(*pParam->nodes)[pos].first  = -1.0;
				(*pParam->nodes)[pos].second = 0.0;
			}
		}
		ReleaseSRWLockExclusive(&gSRWLidarLock[pParam->nThreadID]);
	}

	pParam->pDriver->stop();
	pParam->pDriver->stopMotor();
	RPlidarDriver::DisposeDriver(pParam->pDriver);

	return 0;
}
//-----------------------------------------------------------------------------


// Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
inline void manuallyStartAcquisitionIfNeeded(mvIMPACT::acquire::Device* pDev, const mvIMPACT::acquire::FunctionInterface& fi)
{
	if (pDev->acquisitionStartStopBehaviour.read() == mvIMPACT::acquire::assbUser)
	{
		const mvIMPACT::acquire::TDMR_ERROR result = static_cast<mvIMPACT::acquire::TDMR_ERROR>(fi.acquisitionStart());
		if (result != mvIMPACT::acquire::DMR_NO_ERROR)
		{
			std::cout << "'FunctionInterface.acquisitionStart' returned with an unexpected result: " << result
				      << "(" << mvIMPACT::acquire::ImpactAcquireException::getErrorCodeAsString(result) << ")" << std::endl;
		}
	}
}


// Stop the acquisition manually if this was requested
inline void manuallyStopAcquisitionIfNeeded(mvIMPACT::acquire::Device* pDev, const mvIMPACT::acquire::FunctionInterface& fi)
{
	if (pDev->acquisitionStartStopBehaviour.read() == mvIMPACT::acquire::assbUser)
	{
		const mvIMPACT::acquire::TDMR_ERROR result = static_cast<mvIMPACT::acquire::TDMR_ERROR>(fi.acquisitionStop());
		if (result != mvIMPACT::acquire::DMR_NO_ERROR)
		{
			std::cout << "'FunctionInterface.acquisitionStop' returned with an unexpected result: " << result
				      << "(" << mvIMPACT::acquire::ImpactAcquireException::getErrorCodeAsString(result) << ")" << std::endl;
		}
	}
}

struct CaptureParameter
{
	CaptureParameter(int _threadID, Device* _p, cv::Mat *_buffer)
		: nThreadID(_threadID)
		, pDev(_p)		
		, pBuffer(_buffer)
	{}
	~CaptureParameter() {}
	
	int                     nThreadID;
	Device*                 pDev;
	cv::Mat*                pBuffer;	
};
static CaptureParameter* stPCaptureParams[MAX_NUM_CAMERAS];


//-----------------------------------------------------------------------------
unsigned int __stdcall MVCaptureThread(void* pData)
//-----------------------------------------------------------------------------
{
	CaptureParameter* pParam = reinterpret_cast<CaptureParameter*>(pData);

	if (!pParam->pDev) { return 1; }
	
	mvIMPACT::acquire::FunctionInterface fi(pParam->pDev);

	TDMR_ERROR result = DMR_NO_ERROR;	
	while ((result = static_cast<TDMR_ERROR>(fi.imageRequestSingle())) == DMR_NO_ERROR) {};
	if (result != DEV_NO_FREE_REQUEST_AVAILABLE)
	{
		std::cout << "'FunctionInterface.imageRequestSingle' returned with an unexpected result: " << result
			      << "(" << mvIMPACT::acquire::ImpactAcquireException::getErrorCodeAsString(result) << ")" << std::endl;
	}
	manuallyStartAcquisitionIfNeeded(pParam->pDev, fi);

	// run thread loop
	Request* pRequest = 0;
	Request* pPreviousRequest = 0;
	const unsigned int timeout_ms = 500;
	int requestNr = INVALID_ID;
	cv::Mat matFrame;
	while (gCameraGrabbing[pParam->nThreadID])
	{
		// wait for results from the default capture queue    
		requestNr = fi.imageRequestWaitFor(timeout_ms);
		pRequest  = fi.isRequestNrValid(requestNr) ? fi.getRequest(requestNr) : 0;
		if (pRequest)
		{			
			if (pRequest->isOK())
			{
				int openCVDataType = CV_8UC1;
				switch (pRequest->imagePixelFormat.read())
				{
				case ibpfMono8:
					openCVDataType = CV_8UC1;
					break;
				case ibpfMono10:
				case ibpfMono12:
				case ibpfMono14:
				case ibpfMono16:
					openCVDataType = CV_16UC1;
					break;
				case ibpfMono32:
					openCVDataType = CV_32SC1;
					break;
				case ibpfBGR888Packed:
				case ibpfRGB888Packed:
					openCVDataType = CV_8UC3;
					break;
				case ibpfRGBx888Packed:
					openCVDataType = CV_8UC4;
					break;
				case ibpfRGB101010Packed:
				case ibpfRGB121212Packed:
				case ibpfRGB141414Packed:
				case ibpfRGB161616Packed:
					openCVDataType = CV_16UC3;
					break;
				case ibpfMono12Packed_V1:
				case ibpfMono12Packed_V2:
				case ibpfBGR101010Packed_V2:
				case ibpfRGB888Planar:
				case ibpfRGBx888Planar:
				case ibpfYUV422Packed:
				case ibpfYUV422_10Packed:
				case ibpfYUV422_UYVYPacked:
				case ibpfYUV422_UYVY_10Packed:
				case ibpfYUV422Planar:
				case ibpfYUV444Packed:
				case ibpfYUV444_10Packed:
				case ibpfYUV444_UYVPacked:
				case ibpfYUV444_UYV_10Packed:
				case ibpfYUV444Planar:
				case ibpfYUV411_UYYVYY_Packed:
					printf("[ERROR] Don't know how to render this pixel format (%s)\
                            in OpenCV! Select another one e.g. by writing to \
                            mvIMPACT::acquire::ImageDestination::pixelFormat!\n",
						pRequest->imagePixelFormat.readS().c_str());
					break;
				}				
				
				matFrame = cv::Mat(
					cv::Size(pRequest->imageWidth.read(), pRequest->imageHeight.read()),
					openCVDataType,
					pRequest->imageData.read(),
					pRequest->imageLinePitch.read());

				AcquireSRWLockExclusive(&gSRWLock[pParam->nThreadID]);
				matFrame.copyTo(*pParam->pBuffer);
				ReleaseSRWLockExclusive(&gSRWLock[pParam->nThreadID]);

				//cv::imshow("test", *pParam->pBuffer);
				//cv::waitKey(3);
			}
			else
			{
				std::cout << "Error: " << pRequest->requestResult.readS() << std::endl;
			}
			if (pPreviousRequest)
			{
				pPreviousRequest->unlock();
			}
			pPreviousRequest = pRequest;
			gNumGrabbing++;
			// SuspendThread(GetCurrentThread());
			//::Sleep(10);

			// send a new image request into the capture queue
			fi.imageRequestSingle();
		}
		else
		{
			// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
			// additional information under TDMR_ERROR in the interface reference
			std::cout << "imageRequestWaitFor failed (" 
				      << requestNr << ", " << ImpactAcquireException::getErrorCodeAsString(requestNr) << ")"
				      << ", timeout value too small?" << std::endl;
		}
		::Sleep(10);
	}
	manuallyStopAcquisitionIfNeeded(pParam->pDev, fi);

	if (pRequest) { pRequest->unlock(); }
	// free resources, empty the drivers request queue
	fi.imageRequestReset(0, 0);

	// extract and unlock all requests that are now returned as 'aborted'
	requestNr = INVALID_ID;
	while ((requestNr = fi.imageRequestWaitFor(0)) >= 0)
	{
		pRequest = fi.getRequest(requestNr);
		std::cout << "Request " << requestNr << " did return with status " 
			      << pRequest->requestResult.readS() << std::endl;
		pRequest->unlock();
	}

	return 0;
}
//-----------------------------------------------------------------------------


// CCaptureDatasetDlg dialog
CCaptureDatasetDlg::CCaptureDatasetDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_CAPTUREDATASET_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CCaptureDatasetDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_STATIC_IMAGE_1, m_staticImage[0]);
	DDX_Control(pDX, IDC_STATIC_IMAGE_2, m_staticImage[1]);
}

BEGIN_MESSAGE_MAP(CCaptureDatasetDlg, CDialogEx)
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_DESTROY()
	ON_WM_TIMER()	
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDCANCEL, &CCaptureDatasetDlg::OnBnClickedCancel)	
	ON_BN_CLICKED(IDC_BUTTON_RECORD, &CCaptureDatasetDlg::OnBnClickedButtonRecord)
	ON_BN_CLICKED(IDC_BUTTON_STOP, &CCaptureDatasetDlg::OnBnClickedButtonStop)
END_MESSAGE_MAP()


// CCaptureDatasetDlg message handlers

BOOL CCaptureDatasetDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	CButton *pBtn = (CButton*)GetDlgItem(IDC_BUTTON_STOP);
	pBtn->EnableWindow(FALSE);

	m_bRecord = false;

	for (int camIdx = 0; camIdx < MAX_NUM_CAMERAS; camIdx++)
	{
		gCameraGrabbing[camIdx] = true;
		stPCaptureParams[camIdx] = NULL;
	}

	for (int lidarIdx = 0; lidarIdx < MAX_NUM_LIDARS; lidarIdx++)
	{
		gLidarGrabbing[lidarIdx] = true;
		stPLidarParams[lidarIdx] = NULL;
	}

	//----------------------------MATRIX VISION-----------------------------//
	gNumGrabbing = 0;
	m_numMVCameras = (int)std::min(m_cMVDeviceManager.deviceCount(), (unsigned int)MAX_NUM_CAMERAS);
	for (int camIdx = 0; camIdx < m_numMVCameras; camIdx++)
	{
		try
		{
			Device* pDevice = m_cMVDeviceManager[camIdx];
			assert(NULL != pDevice);

			pDevice->open();
			assert(NULL != pDevice);

			//CameraSettingsBlueCOUGAR cameraSettings(pDevice);
			//cameraSettings.restoreDefault();
			//cameraSettings.autoExposeControl = aecOff;
			//cameraSettings.autoGainControl = agcOff;
			//cameraSettings.gain_dB = 0.0f;
			//cameraSettings.frameRate_Hz = 20.0f;
			//cameraSettings.expose_us = 50000;

			stPCaptureParams[camIdx] = new CaptureParameter(camIdx, pDevice, &m_arrMatFrames[camIdx]);
			m_hCameraThread[camIdx] = (HANDLE)_beginthreadex(0, 0, MVCaptureThread, (LPVOID)(stPCaptureParams[camIdx]), 0, 0);
		}
		catch (mvIMPACT::acquire::ImpactAcquireException &e)
		{
			// this e.g. might happen if the same device is already opened in another process...
			printf("[ERROR] An error occurred while opening the device (error code: %d)\n",
				e.getErrorCode());
		}

		// for visualization
		m_arrMfcCameraImages[camIdx] = NULL;
	}
	//----------------------------MATRIX VISION-----------------------------//


	//-----------------------------RPLIDAR  A2------------------------------//
	gNumLidarGrabbing = 0;
	m_numLidars = MAX_NUM_LIDARS;
	for (int lidarIdx = 0; lidarIdx < m_numLidars; lidarIdx++)
	{
		try
		{
			u_result op_result;
			RPlidarDriver *drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

			if (!drv) 
			{
				fprintf(stderr, "insufficent memory, exit\n");
				exit(-2);
			}

			// make connection...
			if (IS_FAIL(drv->connect(strLidarComPath[lidarIdx], LIDAR_COM_BAUDRATE))) 
			{
				fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
					, strLidarComPath);
				RPlidarDriver::DisposeDriver(drv);
				continue;
			}

			rplidar_response_device_info_t devinfo;

			// retrieving the device info
			////////////////////////////////////////
			op_result = drv->getDeviceInfo(devinfo);

			if (IS_FAIL(op_result)) 
			{
				fprintf(stderr, "Error, cannot get device info.\n");
				RPlidarDriver::DisposeDriver(drv);
				continue;
			}

			// print out the device serial number, firmware and hardware version number..
			printf("RPLIDAR S/N: ");
			for (int pos = 0; pos < 16; ++pos) {
				printf("%02X", devinfo.serialnum[pos]);
			}

			printf("\n"
				"Firmware Ver: %d.%02d\n"
				"Hardware Rev: %d\n"
				, devinfo.firmware_version >> 8
				, devinfo.firmware_version & 0xFF
				, (int)devinfo.hardware_version);


			// check health...
			if (!checkRPLIDARHealth(drv)) 
			{
				RPlidarDriver::DisposeDriver(drv);
				continue;
			}

			drv->startMotor();
			drv->startScan();
			drv->setMotorPWM((_u16)LIDAR_SPEED);

			////////////////////
			m_arrNodes[lidarIdx].resize(LIDAR_RESOLUTION, std::make_pair(0.0, 0.0));
			m_arrNodesBuffer[lidarIdx].resize(LIDAR_RESOLUTION, std::make_pair(0.0, 0.0));
			stPLidarParams[lidarIdx] = new LidarParameter(lidarIdx, drv, &m_arrNodes[lidarIdx]);
			m_hLidarThread[lidarIdx] = (HANDLE)_beginthreadex(0, 0, LidarCaptureThread, (LPVOID)(stPLidarParams[lidarIdx]), 0, 0);
		}
		catch (mvIMPACT::acquire::ImpactAcquireException &e)
		{
			// this e.g. might happen if the same device is already opened in another process...
			printf("[ERROR] An error occurred while opening the device (error code: %d)\n",
				e.getErrorCode());
		}
	}
	//-----------------------------RPLIDAR  A2------------------------------//


	/* start threads */
	::Sleep(30);
	m_bThreadRun = true;
	SetTimer(1000, 30, NULL);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CCaptureDatasetDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}


void CCaptureDatasetDlg::OnClose()
{
	//if (m_bThreadRun)
	//{
	//	for (int camIdx = 0; camIdx < m_numMVCameras; camIdx++)
	//	{
	//		gCameraGrabbing[camIdx] = false;
	//		WaitForSingleObject(m_hCameraThread[camIdx], INFINITE);
	//		CloseHandle(m_hCameraThread[camIdx]);

	//		if (NULL != stPCaptureParams[camIdx])
	//		{
	//			delete stPCaptureParams[camIdx];
	//			stPCaptureParams[camIdx] = NULL;
	//		}
	//	}
	//}

	CDialogEx::OnClose();
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CCaptureDatasetDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CCaptureDatasetDlg::OnDestroy()
{
	CDialogEx::OnDestroy();	
}


void CCaptureDatasetDlg::OnTimer(UINT_PTR nIDEvent)
{
	if (m_bThreadRun)
	{	
		clock_t timeCapture = clock();
		SYSTEMTIME st;
		GetLocalTime(&st);

		/* display */
		for (int camIdx = 0; camIdx < m_numMVCameras; camIdx++)
		{			
			AcquireSRWLockShared(&gSRWLock[camIdx]);
			m_arrMatFrames[camIdx].copyTo(m_arrMatFrameBuffer[camIdx]);
			ReleaseSRWLockShared(&gSRWLock[camIdx]);

			RECT clientRect;
			m_staticImage[camIdx].GetClientRect(&clientRect);
			cv::Size rectSize(clientRect.right, clientRect.bottom);
			cv::Mat  matResizedImage;

			if (m_arrMatFrameBuffer[camIdx].empty()) { continue; }
			cv::cvtColor(m_arrMatFrameBuffer[camIdx], matResizedImage, CV_BGRA2BGR);

			cv::resize(matResizedImage, matResizedImage, rectSize);			
			cv::flip(matResizedImage, matResizedImage, 0);

			if (m_arrMfcCameraImages[camIdx])
			{
				m_arrMfcCameraImages[camIdx]->ReleaseDC();
				delete m_arrMfcCameraImages[camIdx];
				m_arrMfcCameraImages[camIdx] = nullptr;
			}
			m_arrMfcCameraImages[camIdx] = new CImage();
			m_arrMfcCameraImages[camIdx]->Create(rectSize.width, rectSize.height, 24);

			BITMAPINFO bitInfo;
			bitInfo.bmiHeader.biBitCount = 24;
			bitInfo.bmiHeader.biWidth = rectSize.width;
			bitInfo.bmiHeader.biHeight = rectSize.height;
			bitInfo.bmiHeader.biPlanes = 1;
			bitInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
			bitInfo.bmiHeader.biCompression = BI_RGB;
			bitInfo.bmiHeader.biClrImportant = 0;
			bitInfo.bmiHeader.biClrUsed = 0;
			bitInfo.bmiHeader.biSizeImage = 0;
			bitInfo.bmiHeader.biXPelsPerMeter = 0;
			bitInfo.bmiHeader.biYPelsPerMeter = 0;

			StretchDIBits(m_arrMfcCameraImages[camIdx]->GetDC(), 0, 0,
				rectSize.width, rectSize.height, 0, 0,
				rectSize.width, rectSize.height,
				matResizedImage.data, &bitInfo, DIB_RGB_COLORS, SRCCOPY
			);

			m_arrMfcCameraImages[camIdx]->BitBlt(::GetDC(m_staticImage[camIdx].m_hWnd), 0, 0);
			m_arrMfcCameraImages[camIdx]->ReleaseDC();
			delete m_arrMfcCameraImages[camIdx];
			m_arrMfcCameraImages[camIdx] = nullptr;
		}

		for (int lidarIdx = 0; lidarIdx < m_numLidars; lidarIdx++)
		{
			AcquireSRWLockShared(&gSRWLidarLock[lidarIdx]);
			m_arrNodesBuffer[lidarIdx] = m_arrNodes[lidarIdx];
			ReleaseSRWLockShared(&gSRWLidarLock[lidarIdx]);

			//cv::Mat matLidar = cv::Mat::zeros((int)LIDAR_MAP_SIZE, (int)LIDAR_MAP_SIZE, CV_8UC3);
			//std::vector<cv::Point2f> distancePoints;
			//distancePoints.reserve(LIDAR_RESOLUTION);

			//std::sort(m_arrNodesBuffer[lidarIdx].begin(), m_arrNodesBuffer[lidarIdx].end(), pairCompare);
			//for (int pos = 0; pos < LIDAR_RESOLUTION; pos++)
			//{
			//	if (0 > m_arrNodesBuffer[lidarIdx][pos].first) { break;	}

			//	double angle    = m_arrNodesBuffer[lidarIdx][pos].first;
			//	double distance = m_arrNodesBuffer[lidarIdx][pos].second * LIDAR_RESCALE;
			//	distancePoints.push_back(cv::Point2f(
			//			distance * cos(angle) + (double)LIDAR_MAP_SIZE * 0.5,
			//			distance * sin(angle) + (double)LIDAR_MAP_SIZE * 0.5));
			//}

			//if (0 == distancePoints.size()) { continue; }

			//distancePoints.push_back(distancePoints.front());

			//for (int pointIdx = 0; pointIdx < distancePoints.size() - 1; pointIdx++)
			//{
			//	cv::line(matLidar, distancePoints[pointIdx], distancePoints[pointIdx + 1], cv::Scalar(0, 255, 0));
			//	int a = 0;
			//}
			//int b = 0;
		}

		/* file saving */
		if (m_bRecord)
		{
			for (int camIdx = 0; camIdx < m_numMVCameras; camIdx++)
			{
				std::string strSavePath = std::string(SAVE_PATH) + "/" + "View_00" + std::to_string(camIdx + 1);
				std::wstring wideStrDirName = L"";
				wideStrDirName.assign(strSavePath.begin(), strSavePath.end());
				if (CreateDirectory(wideStrDirName.c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError())
				{
					char strFileName[100];
					sprintf_s(strFileName, "%s/frame_%02d%02d%02d%02d%02d%03d.jpg", strSavePath.c_str(),
						st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
					cv::imwrite(strFileName, m_arrMatFrameBuffer[camIdx]);
				}				
			}

			for (int sensorIdx = 0; sensorIdx < m_numLidars; sensorIdx++)
			{
				std::string strSavePath = std::string(SAVE_PATH) + "/" + "Lidar_00" + std::to_string(sensorIdx + 1);
				std::wstring wideStrDirName = L"";
				wideStrDirName.assign(strSavePath.begin(), strSavePath.end());
				if (CreateDirectory(wideStrDirName.c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError())
				{
					char strFileName[100];
					sprintf_s(strFileName, "%s/frame_%02d%02d%02d%02d%02d%03d.txt", strSavePath.c_str(),
						st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

					try
					{
						FILE *fp = NULL;
						fopen_s(&fp, strFileName, "w");

						for (int pos = 0; pos < m_arrNodesBuffer[sensorIdx].size(); pos++)
						{
							if (0.0 > m_arrNodesBuffer[sensorIdx][pos].first) { break; }
							fprintf_s(fp, "%lf,%lf\n",
								m_arrNodesBuffer[sensorIdx][pos].first,
								m_arrNodesBuffer[sensorIdx][pos].second);
						}

						fclose(fp);
					}
					catch (int e)
					{

					}
				}
			}
		}	
	}

	CDialogEx::OnTimer(nIDEvent);
}


void CCaptureDatasetDlg::OnBnClickedCancel()
{
	if (m_bThreadRun)
	{
		for (int camIdx = 0; camIdx < m_numMVCameras; camIdx++)
		{
			gCameraGrabbing[camIdx] = false;
			WaitForSingleObject(m_hCameraThread[camIdx], INFINITE);
			CloseHandle(m_hCameraThread[camIdx]);

			if (NULL != stPCaptureParams[camIdx])
			{
				delete stPCaptureParams[camIdx];
				stPCaptureParams[camIdx] = NULL;
			}
		}

		for (int lidarIdx = 0; lidarIdx < m_numLidars; lidarIdx++)
		{
			gLidarGrabbing[lidarIdx] = false;
			WaitForSingleObject(m_hLidarThread[lidarIdx], INFINITE);
			CloseHandle(m_hLidarThread[lidarIdx]);

			if (NULL != stPLidarParams[lidarIdx])
			{
				delete stPLidarParams[lidarIdx];
				stPLidarParams[lidarIdx] = NULL;
			}
		}
	}

	CDialogEx::OnCancel();
}


void CCaptureDatasetDlg::OnBnClickedButtonRecord()
{
	m_timeStart = clock();
	m_bRecord = true;
	CButton *pBtn = (CButton*)GetDlgItem(IDC_BUTTON_STOP);
	pBtn->EnableWindow(TRUE);
	pBtn = (CButton*)GetDlgItem(IDC_BUTTON_RECORD);
	pBtn->EnableWindow(FALSE);
}


void CCaptureDatasetDlg::OnBnClickedButtonStop()
{
	m_bRecord = false;
	CButton *pBtn = (CButton*)GetDlgItem(IDC_BUTTON_STOP);
	pBtn->EnableWindow(FALSE);
	pBtn = (CButton*)GetDlgItem(IDC_BUTTON_RECORD);
	pBtn->EnableWindow(TRUE);
}
