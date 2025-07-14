
#ifndef Unit1H
#define Unit1H

#define VC_EXTRALEAN
#define WIN32_EXTRA_LEAN
#define WIN32_LEAN_AND_MEAN

#include <Forms.hpp>
#include <Buttons.hpp>
#include <Menus.hpp>
#include <Classes.hpp>
#include <ComCtrls.hpp>
#include <Controls.hpp>
#include <Dialogs.hpp>
#include <ExtCtrls.hpp>
#include <StdCtrls.hpp>
#include <Buttons.hpp>

// GDI+ system
//#include <Unknwn.h>
//#include <windows.h>
//#include <algorithm>
//using std::min;
//using std::max;
#pragma option push
	#pragma warn -8022
	#include <gdiplus.h>
	#pragma link "gdiplus.lib"
	//#pragma comment (lib, "gdiplus.lib")
#pragma option pop

#include <vector>
#include <stdint.h>
#include <stdio.h>

#include "SerialPort.h"
#include "HighResolutionTick.h"
#include "CriticalSection.h"

// ******************************************************************************

#define WM_INIT_GUI           (WM_USER + 100)
#define WM_CONNECT            (WM_USER + 101)
#define WM_DISCONNECT         (WM_USER + 102)
#define WM_BREAK              (WM_USER + 103)

#define ARRAY_SIZE(array)     (sizeof(array) / sizeof((array)[0]))

#define DEFAULT_SERIAL_SPEED  921600

#define DMA_ADC_DATA_LENGTH   128           // must be 2^n (2, 4, 8 .. 1024 etc)

#pragma option push
#pragma warn -8027

typedef struct
{
	bool ready;

	struct {
		unsigned int        buffer_wr;
		uint8_t             buffer[16384];
		CHighResolutionTick timer;
	} rx;

#if 0
	struct {
		unsigned int        buffer_wr;
		uint8_t             buffer[512];
		CHighResolutionTick timer;
	} tx;
#endif

} t_client;

#pragma pack(push, 1)
typedef struct {
	union {
		uint32_t marker;
		uint8_t  marker_b[sizeof(uint32_t)];
	};
	union {
		uint16_t crc;
		uint8_t  crc_b[sizeof(uint16_t)];
	};
	union {
		float   data[DMA_ADC_DATA_LENGTH * 8];
		uint8_t data_b[sizeof(float) * DMA_ADC_DATA_LENGTH * 8];
	};
} t_packet;
#pragma pack(pop)

typedef struct {
	float freq;
	float coeff;
	float sin;
	float cos;
	float re;
	float im;
} t_goertzel;

// ******************************************************************************

typedef void __fastcall (__closure *mainForm_threadProcess)();

class CThread : public TThread
{
private:
	mainForm_threadProcess m_process;
	bool                   m_sync;
	DWORD                  m_sleep_ms;
	HANDLE                 m_mutex;

protected:
	void __fastcall Execute()
	{
		while (!Terminated)
		{
			if (m_mutex)
			{
				const DWORD res = ::WaitForSingleObject(m_mutex, m_sleep_ms);
				if (res == WAIT_OBJECT_0)
					::ReleaseMutex(m_mutex);
			}
			else
				Sleep(m_sleep_ms);
			if (m_process == NULL)
				continue;
			if (!m_sync)
				m_process();
			else
				Synchronize(m_process);
		}
		ReturnValue = 0;
	}

public:
	__fastcall CThread(mainForm_threadProcess process, TThreadPriority priority, DWORD sleep_ms, bool start, bool sync) : TThread(!start)
	{
		m_sleep_ms = sleep_ms;
		m_process  = process;
		m_sync     = sync;

		FreeOnTerminate = false;
		Priority        = priority;

		m_mutex = ::CreateMutex(NULL, TRUE, NULL);
	}

	virtual __fastcall ~CThread()
	{
		m_process = NULL;

		if (m_mutex)
		{
			::WaitForSingleObject(m_mutex, 100);		// wait for upto 100ms
			::CloseHandle(m_mutex);
			m_mutex = NULL;
		}
	}

	__property bool   sync    = {read = m_sync,     write = m_sync};
	__property DWORD  sleepMS = {read = m_sleep_ms, write = m_sleep_ms};
	__property HANDLE mutex   = {read = m_mutex};
};

// ******************************************************************************

class TForm1 : public TForm
{
__published:
	TStatusBar *StatusBar1;
	TOpenDialog *OpenDialog1;
	TSaveDialog *SaveDialog1;
	TTimer *Timer1;
	TPanel *Panel2;
	TLabel *Label1;
	TLabel *Label2;
	TComboBox *SerialPortComboBox;
	TComboBox *SerialSpeedComboBox;
	TSpeedButton *ConsoleWindowSpeedButton;
	TPaintBox *PaintBox1;
	TSpeedButton *PauseSpeedButton;
	TButton *CaptureButton;
	TTrackBar *NormaliseTrackBar;
	TSpeedButton *HistogramSpeedButton;
	void __fastcall FormCreate(TObject *Sender);
	void __fastcall FormDestroy(TObject *Sender);
	void __fastcall FormClose(TObject *Sender, TCloseAction &Action);
	void __fastcall SerialPortComboBoxDropDown(TObject *Sender);
	void __fastcall FormKeyDown(TObject *Sender, WORD &Key,
			 TShiftState Shift);
	void __fastcall Timer1Timer(TObject *Sender);
	void __fastcall SerialPortComboBoxSelect(TObject *Sender);
	void __fastcall SerialSpeedComboBoxSelect(TObject *Sender);
	void __fastcall StatusBar1Resize(TObject *Sender);
	void __fastcall PaintBox1Paint(TObject *Sender);
	void __fastcall ConsoleWindowSpeedButtonClick(TObject *Sender);
	void __fastcall CaptureButtonClick(TObject *Sender);
	void __fastcall NormaliseTrackBarChange(TObject *Sender);
	void __fastcall HistogramSpeedButtonClick(TObject *Sender);

private:

	String m_ini_filename;

	Gdiplus::GdiplusStartupInput m_gdiplusStartupInput;
	ULONG_PTR                    m_gdiplusToken;

	bool m_opening;
	bool m_closing;

	struct
	{
		bool  created;
		HWND  handle;
		RECT  rect;
		FILE *std_out;
		FILE *std_err;
		FILE *std_in;
	} m_console;

	struct {
		int         width;
		int         height;
		SYSTEM_INFO system_info;
	} m_screen;

	std::vector <uint8_t> m_beep1_wav;
	std::vector <uint8_t> m_beep2_wav;

	struct {
		CCriticalSectionObj cs;
		CThread            *thread;
	} m_thread;

	struct {
		CSerialPort port;
		t_client    client;
	} m_serial;

	Graphics::TBitmap *m_bitmap_main;

	int m_breaks;

	unsigned int m_frames;
	float        m_values[8][DMA_ADC_DATA_LENGTH];

	// one for each waveform
	struct {
		t_goertzel goertzel;
		float        magnitude_rms;
		float        phase_deg;
		float        average;
		unsigned int m_histogram[256];
	} m_waveform_info[8];

	void __fastcall loadSettings();
	void __fastcall saveSettings();

	void __fastcall comboBoxAutoWidth(TComboBox *comboBox);

	void __fastcall updateSerialPortCombo();

	void __fastcall serialDisconnect();
	bool __fastcall serialConnect();

	void __fastcall processClient(t_client &client, CSerialPort *serial_port);
	void __fastcall processSerial();
	void __fastcall threadProcess();

	void __fastcall drawArc(Graphics::TBitmap *bm, const int x, const int y, const float start_angle, const float range_angle, const float radius);
	void __fastcall drawCircle(Graphics::TBitmap *bm, const int x, const int y, const int radius);

	//void __fastcall addMemoLine(String s);

	void __fastcall WMWindowPosChanging(TWMWindowPosChanging &msg);
	void __fastcall CMMouseEnter(TMessage &msg);
	void __fastcall CMMouseLeave(TMessage &msg);
	void __fastcall WMInitGUI(TMessage &msg);
	void __fastcall WMConnect(TMessage &msg);
	void __fastcall WMDisconnect(TMessage &msg);
	void __fastcall WMBreak(TMessage &msg);

protected:

	#pragma option push -vi-
	BEGIN_MESSAGE_MAP
		VCL_MESSAGE_HANDLER(WM_WINDOWPOSCHANGING, TWMWindowPosMsg, WMWindowPosChanging);

		VCL_MESSAGE_HANDLER(CM_MOUSELEAVE,        TMessage, CMMouseLeave);
		VCL_MESSAGE_HANDLER(CM_MOUSEENTER,        TMessage, CMMouseEnter);

		VCL_MESSAGE_HANDLER(WM_INIT_GUI,          TMessage, WMInitGUI);

		VCL_MESSAGE_HANDLER(WM_CONNECT,           TMessage, WMConnect);
		VCL_MESSAGE_HANDLER(WM_DISCONNECT,        TMessage, WMDisconnect);
		VCL_MESSAGE_HANDLER(WM_BREAK,             TMessage, WMBreak);

	END_MESSAGE_MAP(TForm)
	#pragma option pop

public:
	__fastcall TForm1(TComponent* Owner);
};

extern PACKAGE TForm1 *Form1;

#pragma option pop

#endif

