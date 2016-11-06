
// CaptureDatasetDlg.h : header file
//

#pragma once
#include "afxwin.h"
#include <time.h>
#include <opencv2\opencv.hpp>
#include <mvIMPACT_CPP\mvIMPACT_acquire.h>

#define NUM_CAMERAS (2)

// CCaptureDatasetDlg dialog
class CCaptureDatasetDlg : public CDialogEx
{
// Construction
public:
	CCaptureDatasetDlg(CWnd* pParent = NULL);	// standard constructor
	afx_msg void OnClose();
	afx_msg void OnDestroy();
	afx_msg void OnTimer(UINT_PTR nIDEvent);	
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedButtonRecord();
	afx_msg void OnBnClickedButtonStop();
// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CAPTUREDATASET_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnPaint();	
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CStatic m_staticImage[NUM_CAMERAS];

private:
	cv::Mat m_arrMatFrames[NUM_CAMERAS];
	cv::Mat m_arrMatFrameBuffer[NUM_CAMERAS];
	CImage *m_arrMfcImages[NUM_CAMERAS];
	HANDLE  m_hThread[NUM_CAMERAS];
	bool    m_bThreadRun;
	bool    m_bRecord;
	int     m_numMVCameras;	
	int     m_MVOpenCVDataType;
	DeviceManager m_cMVDeviceManager;
	clock_t m_timeStart;
};
