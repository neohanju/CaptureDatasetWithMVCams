
// CaptureDatasetDlg.cpp : implementation file
//

#include "stdafx.h"
#include "CaptureDataset.h"
#include "CaptureDatasetDlg.h"
#include "afxdialogex.h"
#include <thread>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define SAVE_PATH "D:/Workspace/Dataset/Record_MVCam"

using namespace mvIMPACT::acquire;

static volatile int  gNumGrabbing = 0;
static volatile bool gCameraGrabbing[NUM_CAMERAS] = { true, true };
static cv::Mat       gMatFrames[NUM_CAMERAS];
static SRWLOCK       gSRWLock[NUM_CAMERAS];

//-----------------------------------------------------------------------------
// Start the acquisition manually if this was requested(this is to prepare the driver for data capture and tell the device to start streaming data)
inline void manuallyStartAcquisitionIfNeeded(mvIMPACT::acquire::Device* pDev, const mvIMPACT::acquire::FunctionInterface& fi)
//-----------------------------------------------------------------------------
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

//-----------------------------------------------------------------------------
// Stop the acquisition manually if this was requested
inline void manuallyStopAcquisitionIfNeeded(mvIMPACT::acquire::Device* pDev, const mvIMPACT::acquire::FunctionInterface& fi)
//-----------------------------------------------------------------------------
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
static CaptureParameter* stPCaptureParams[NUM_CAMERAS] = { NULL, NULL };

unsigned int __stdcall MVCaptureThread(void* pData)
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

	//----------------------------MATRIX VISION-----------------------------//
	gNumGrabbing = 0;
	m_numMVCameras = m_cMVDeviceManager.deviceCount();
	for (int camIdx = 0; camIdx < m_numMVCameras; camIdx++)
	{
		try
		{
			Device* pDevice = m_cMVDeviceManager[camIdx];
			assert(NULL != pDevice);

			pDevice->open();
			assert(NULL != pDevice);

			stPCaptureParams[camIdx] = new CaptureParameter(camIdx, pDevice, &m_arrMatFrames[camIdx]);
			m_hThread[camIdx] = (HANDLE)_beginthreadex(0, 0, MVCaptureThread, (LPVOID)(stPCaptureParams[camIdx]), 0, 0);
		}
		catch (mvIMPACT::acquire::ImpactAcquireException &e)
		{
			// this e.g. might happen if the same device is already opened in another process...
			printf("[ERROR] An error occurred while opening the device (error code: %d)\n",
				e.getErrorCode());
		}
	}
	//----------------------------MATRIX VISION-----------------------------//

	for (int imgIdx = 0; imgIdx < NUM_CAMERAS; imgIdx++)
	{
		m_arrMfcImages[imgIdx] = NULL;
	}
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
	//		WaitForSingleObject(m_hThread[camIdx], INFINITE);
	//		CloseHandle(m_hThread[camIdx]);

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
			RECT clientRect;
			m_staticImage[camIdx].GetClientRect(&clientRect);
			cv::Size rectSize(clientRect.right, clientRect.bottom);
			cv::Mat  matResizedImage;

			AcquireSRWLockShared(&gSRWLock[camIdx]);
			m_arrMatFrames[camIdx].copyTo(m_arrMatFrameBuffer[camIdx]);
			ReleaseSRWLockShared(&gSRWLock[camIdx]);

			if (m_arrMatFrameBuffer[camIdx].empty()) { continue; }
			cv::cvtColor(m_arrMatFrameBuffer[camIdx], matResizedImage, CV_BGRA2BGR);

			cv::resize(matResizedImage, matResizedImage, rectSize);			
			cv::flip(matResizedImage, matResizedImage, 0);

			if (m_arrMfcImages[camIdx])
			{
				m_arrMfcImages[camIdx]->ReleaseDC();
				delete m_arrMfcImages[camIdx];
				m_arrMfcImages[camIdx] = nullptr;
			}
			m_arrMfcImages[camIdx] = new CImage();
			m_arrMfcImages[camIdx]->Create(rectSize.width, rectSize.height, 24);

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

			StretchDIBits(m_arrMfcImages[camIdx]->GetDC(), 0, 0,
				rectSize.width, rectSize.height, 0, 0,
				rectSize.width, rectSize.height,
				matResizedImage.data, &bitInfo, DIB_RGB_COLORS, SRCCOPY
			);

			m_arrMfcImages[camIdx]->BitBlt(::GetDC(m_staticImage[camIdx].m_hWnd), 0, 0);
			m_arrMfcImages[camIdx]->ReleaseDC();
			delete m_arrMfcImages[camIdx];
			m_arrMfcImages[camIdx] = nullptr;
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
			WaitForSingleObject(m_hThread[camIdx], INFINITE);
			CloseHandle(m_hThread[camIdx]);

			if (NULL != stPCaptureParams[camIdx])
			{
				delete stPCaptureParams[camIdx];
				stPCaptureParams[camIdx] = NULL;
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
