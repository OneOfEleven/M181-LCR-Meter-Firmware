
// 1o11

#include <vcl.h>
#include <inifiles.hpp>
#include <Clipbrd.hpp>

#include <stdio.h>
//#incluve <limits.h>
#include <float.h>
//#include <fastmath.h>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <mmsystem.h>		// PlaySound()
//#include <stdlib.h>     // srand, rand

#pragma hdrstop

#include "Unit1.h"
#include "crc.h"

#pragma package(smart_init)
#pragma resource "*.dfm"

#define MIN_WAV_SIZE             44 // bytes

#define RAD_TO_DEG              ((float)(180.0 / M_PI))
#define DEG_TO_RAD              ((float)(M_PI / 180.0))

#define SQR(x)                  ((x) * (x))

#define PACKET_MARKER           0x19621996

TForm1 *Form1 = NULL;

// **************************************************************

#if 1
	float phase_diff(const float phase_deg_1, const float phase_deg_2)
	{
		float ph1_re;
		float ph1_im;
		float ph2_re;
		float ph2_im;
		float d_re;
		float d_im;

		{
			const float phase_rad = phase_deg_1 * DEG_TO_RAD;
			ph1_re = cos(phase_rad);
			ph1_im = sin(phase_rad);
		}

		{
			const float phase_rad = phase_deg_2 * DEG_TO_RAD;
			ph2_re = cos(phase_rad);
			ph2_im = sin(phase_rad);
		}

		// conj multiply
		d_re = (ph1_re * ph2_re) + (ph1_im * ph2_im);
		d_im = (ph1_re * ph2_im) - (ph1_im * ph2_re);

		// phase
		const float phase_deg = (d_re != 0) ? atan2(d_im, d_re) * RAD_TO_DEG : _FPCLASS_QNAN;

		return phase_deg;
	}
#else
	float phase_diff(float phase_deg_1, float phase_deg_2)
	{
		while (phase_deg_1 < 0) phase_deg_1 += 360.0f;
		while (phase_deg_2 < 0) phase_deg_2 += 360.0f;

		// return shortest angle difference - clockwise or anti-clockwise
		#if 1
			float deg = (phase_deg_1 - phase_deg_2) + 360.0f + 180.0f;
			while (deg >= 360.0f) deg -= 360.0f;
			//while (deg <    0.0f) deg += 360.0f;
			return deg - 180.0f;
		#else
			return fmodf((phase_deg_1 - phase_deg_2) + 360.0f + 180.0f, 360.0f) - 180.0f;
		#endif
	}
#endif

// **************************************************************

void __fastcall goertzel_block(const float *samples, const unsigned int len, t_goertzel *g)
{
	float m1 = 0;
	float m2 = 0;

	for (unsigned int i = 0; i < len; i++)
	{
		const float m = samples[i] + (g->coeff * m1) - m2;
		m2 = m1;
		m1 = m;
	}

	g->re = (m1 * g->cos) - m2;
	g->im = -m1 * g->sin;

	// normalise the output amplitude
	const float scale = 2.0f / len;
	g->re *= scale;
	g->im *= scale;
}

void __fastcall goertzel_init(t_goertzel *g, const float normalized_freq)
{
	const float w  = 2.0 * M_PI * normalized_freq;
	const float wr = cos(w);
	const float wi = sin(w);

	g->freq  = normalized_freq;
	g->coeff = 2.0f * wr;
	g->cos   = wr;
	g->sin   = wi;
	g->re    = 0;
	g->im    = 0;
}

// **************************************************************

String __fastcall getLastErrorStr(const DWORD err)
{
	String s(_T("unknown error"));
	char *buf = NULL;

	const WORD prevErrorMode = ::SetErrorMode(SEM_FAILCRITICALERRORS);	// ignore critical errors

	const HMODULE wnet_handle = ::GetModuleHandle(_T("wininet.dll"));

	DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_FROM_SYSTEM;
	if (wnet_handle)
		flags |= FORMAT_MESSAGE_FROM_HMODULE;		// retrieve message from specified DLL

	const DWORD res = ::FormatMessage(flags, wnet_handle, err, 0, (LPSTR)&buf, 0, NULL);
	(void)res;

	if (wnet_handle)
		::FreeLibrary(wnet_handle);

	if (buf)
	{
		int len = (int)strlen(buf) - 1;
		while (len >= 0 && (buf[len] == '\r' || buf[len] == '\n' || buf[len] == '.'))
			buf[len--] = '\0';

		//str.printf("[%d] %s", err, buf);
		s = String(buf);

		::LocalFree(buf);
	}

	::SetErrorMode(prevErrorMode);

	return s;
}

double __fastcall degDifference(double deg1, double deg2)
{
	while (deg1 < 0) deg1 += 360;
	while (deg2 < 0) deg2 += 360;

	// return shortest angle difference - clockwise or anti-clockwise
	#if 1
		double deg = (deg1 - deg2) + 360 + 180;
		while (deg >= 360) deg -= 360;
		return deg - 180;
	#else
		return fmod((deg1 - deg2) + 360 + 180, 360) - 180;
	#endif
}

int __fastcall fetchResource(String name, std::vector <uint8_t> &buffer)
{
	buffer.resize(0);

	const HMODULE handle = GetModuleHandle(NULL);
	if (handle == NULL)
		return -1;

	const HRSRC res_info = FindResource(handle, name.c_str(), RT_RCDATA);
	if (res_info == NULL)
		return -2;

	const HGLOBAL res_data = LoadResource(handle, res_info);
	if (res_data == NULL)
		return -3;

	const DWORD dwSize = SizeofResource(handle, res_info);
	if (dwSize == 0)
		return -4;

	const VOID *p_res_data = LockResource(res_data);
	if (p_res_data == NULL)
		return -5;

	buffer.resize(dwSize);

	memmove(&buffer[0], p_res_data, dwSize);

	return (int)dwSize;
}

// ************************************************************************

typedef struct
{
	uint16_t MajorVer;
	uint16_t MinorVer;
	uint16_t ReleaseVer;
	uint16_t BuildVer;
} TVersion;

bool __fastcall getBuildInfo(String filename, TVersion *version)
{
	DWORD ver_info_size;
	char *ver_info;
	UINT buffer_size;
	LPVOID buffer;
	DWORD dummy;

	if (version == NULL || filename.IsEmpty())
		return false;

	memset(version, 0, sizeof(TVersion));

	ver_info_size = ::GetFileVersionInfoSizeA(filename.c_str(), &dummy);
	if (ver_info_size == 0)
		return false;

	ver_info = new char [ver_info_size];
	if (ver_info == NULL)
		return false;

	if (::GetFileVersionInfoA(filename.c_str(), 0, ver_info_size, ver_info) == FALSE)
	{
		delete [] ver_info;
		return false;
	}

	if (::VerQueryValue(ver_info, _T("\\"), &buffer, &buffer_size) == FALSE)
	{
		delete [] ver_info;
		return false;
	}

	PVSFixedFileInfo ver = (PVSFixedFileInfo)buffer;
	version->MajorVer   = (ver->dwFileVersionMS >> 16) & 0xFFFF;
	version->MinorVer   = (ver->dwFileVersionMS >>  0) & 0xFFFF;
	version->ReleaseVer = (ver->dwFileVersionLS >> 16) & 0xFFFF;
	version->BuildVer   = (ver->dwFileVersionLS >>  0) & 0xFFFF;

	delete [] ver_info;

	return true;
}

// *******************************************

std::vector <String> parseString(String s, String delimiter)
{
	std::vector <String> params;

	if (delimiter.Pos('\t') <= 0)
	{	// replace any tabs with spaces
		int p;
		while ((p = s.Pos('\t')) > 0)
			s[p] = ' ';
	}

	s = s.Trim();

	// in borland Strings, the 1st char is at index '1' .. mr borland tried (but failed) to break the C language
	while (!s.IsEmpty())
	{
		const int p = s.Pos(delimiter);

		if (p <= 0)
		{	// no more found
			params.push_back(s);
			break;
		}

		if (p == 1)
		{	// found at start of string
			s = s.SubString(p + delimiter.Length(), s.Length()).Trim();
			continue;
		}

		// found part way along string
		params.push_back(s.SubString(1, p - 1).Trim());

		s = s.SubString(p + delimiter.Length(), s.Length()).Trim();
	}

	return params;
}

// ************************************************************************

void strTrim(char *s)
{
	if (s == NULL)
		return;
	if (s[0] == '\0')
		return;

	char *e = s + strlen(s);

	char *p = s;
	while (*p == ' ' || *p == '\t')
		p++;
	memmove(s, p, (e - p) + 1);         // remove leading spaces

	e -= (p - s) + 1;
	while (e > s && (*e == ' ' || *e == '\t'))
		*e-- = '\0';                    // remove trailing spaces
}

__fastcall TForm1::TForm1(TComponent* Owner)
	: TForm(Owner)
{
}

void __fastcall TForm1::FormCreate(TObject *Sender)
{
	m_opening = true;
	m_closing = false;

//	{
//		char s[] = "  12 345 6 789  ";
//		strTrim(s);
//	}

	{
		TVersion version;
		getBuildInfo(Application->ExeName, &version);

		String s;
		s.printf(" v%u.%u.%u", version.MajorVer, version.MinorVer, version.ReleaseVer);
		#ifdef _DEBUG
			s += ".debug";
		#endif
		s += " 1o11 ";
		this->Caption = Application->Title + s;
	}

	m_gdiplusToken = 0;
	Gdiplus::GdiplusStartup(&m_gdiplusToken, &m_gdiplusStartupInput, NULL);

	{	// create a console window and redirect all stdout and stderr (printf's) to it

		m_console.created     = false;
		m_console.handle      = NULL;
		m_console.rect.top    = 0;
		m_console.rect.left   = 0;
		m_console.rect.right  = 0;
		m_console.rect.bottom = 0;
		m_console.std_out     = NULL;
		m_console.std_err     = NULL;
		m_console.std_in      = NULL;

		#if 1
			m_console.created = ::AllocConsole();
			if (m_console.created)
			{
				m_console.handle = ::GetConsoleWindow();
				if (m_console.handle)
				{
					{	// remove the conso;e window top bar icons/buttons
						const DWORD style = ::GetWindowLong(m_console.handle, GWL_STYLE) & ~(WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU);
						::SetWindowLong(m_console.handle, GWL_STYLE, style);
						::SetWindowPos(m_console.handle, NULL, 0, 0, 0, 0, SWP_NOSIZE | SWP_NOMOVE | SWP_FRAMECHANGED);
					}

					// std_out = freopen("myfile.txt", "w", stdout);
					// freopen_s(&std_out, "CONOUT$", "w", stdout);
					// freopen_s(&std_out, "CON", "w", stdout);

					m_console.std_out = freopen("CONOUT$", "w", stdout);
					m_console.std_err = freopen("CONOUT$", "w", stderr);
					m_console.std_in  = freopen("CONOUT$", "r", stdin);    // Note: "r", not "w"

					//std::cout << "console window" << std::endl;

					::GetWindowRect(m_console.handle, &m_console.rect);
					::SetWindowPos(m_console.handle, HWND_TOP, m_console.rect.left, m_console.rect.top, m_console.rect.right - m_console.rect.left, m_console.rect.bottom - m_console.rect.top, SWP_SHOWWINDOW + SWP_ASYNCWINDOWPOS);
				}
			}

			printf("%s\n\n", Caption.c_str());
		#endif
	}

	{
//		char username[64];
//		DWORD size = sizeof(username);
//		if (::GetUserNameA(username, &size) != FALSE && size > 1)
//			m_ini_filename = ChangeFileExt(Application->ExeName, "_" + String(username) + ".ini");
//		else
			m_ini_filename = ChangeFileExt(Application->ExeName, ".ini");
	}

	{
		::GetSystemInfo(&m_screen.system_info);
		printf(	"      OEM id: %u\n"
					"num of cpu's: %u\n"
					"   page size: %u\n"
					"    cpu type: %u\n"
					"min app addr: %lx\n"
					"max app addr: %lx\n"
					"active cpu mask: %u\n\n",
					m_screen.system_info.dwOemId,
					m_screen.system_info.dwNumberOfProcessors,
					m_screen.system_info.dwPageSize,
					m_screen.system_info.dwProcessorType,
					m_screen.system_info.lpMinimumApplicationAddress,
					m_screen.system_info.lpMaximumApplicationAddress,
					m_screen.system_info.dwActiveProcessorMask);
	}

	{	// the screen size is the phsyical screen size
		m_screen.width  = 0;
		m_screen.height = 0;
		HDC hDC = GetDC(0);
		if (hDC != NULL)
		{
			//m_screen.screen_bits_per_pixel = ::GetDeviceCaps(hDC, BITSPIXEL);
			m_screen.width  = ::GetDeviceCaps(hDC, HORZRES);
			m_screen.height = ::GetDeviceCaps(hDC, VERTRES);
			ReleaseDC(0, hDC);
			printf("screen %d %d\n\n", m_screen.width, m_screen.height);
		}
	}

	this->DoubleBuffered  = true;
	Panel2->DoubleBuffered = true;
//	Memo1->DoubleBuffered = true;

	PaintBox1->ControlStyle = PaintBox1->ControlStyle << csOpaque;

//	Memo1->Clear();

	m_bitmap_main = NULL;

	m_thread.thread = NULL;

	OpenDialog1->InitialDir = ExtractFilePath(Application->ExeName);
	SaveDialog1->InitialDir = ExtractFilePath(Application->ExeName);

	fetchResource("BEEP1_WAV", m_beep1_wav);
	fetchResource("BEEP2_WAV", m_beep2_wav);

	memset(&m_values, 0, sizeof(m_values));

	for (unsigned int i = 0; i < ARRAY_SIZE(m_waveform_info); i++)
	{
		goertzel_init(&m_waveform_info[i].goertzel, 2.0 / ARRAY_SIZE(m_values[i]));
		m_waveform_info[i].magnitude_rms = 0;
		m_waveform_info[i].phase_deg     = 0;
		memset(m_waveform_info[i].histogram, 0, sizeof(m_waveform_info[i].histogram));
	}

	// ******************

	{
		updateSerialPortCombo();

		const TNotifyEvent ne = SerialSpeedComboBox->OnChange;
		SerialSpeedComboBox->OnChange = NULL;
		SerialSpeedComboBox->Clear();
		SerialSpeedComboBox->AddItem("300",     (TObject *)300);
		SerialSpeedComboBox->AddItem("600",     (TObject *)600);
		SerialSpeedComboBox->AddItem("1200",    (TObject *)1200);
		SerialSpeedComboBox->AddItem("2400",    (TObject *)2400);
		SerialSpeedComboBox->AddItem("4800",    (TObject *)4800);
		SerialSpeedComboBox->AddItem("9600",    (TObject *)9600);
		SerialSpeedComboBox->AddItem("19200",   (TObject *)19200);
		SerialSpeedComboBox->AddItem("38400",   (TObject *)38400);
		SerialSpeedComboBox->AddItem("57600",   (TObject *)57600);
		SerialSpeedComboBox->AddItem("76800",   (TObject *)76800);
		SerialSpeedComboBox->AddItem("115200",  (TObject *)115200);
		SerialSpeedComboBox->AddItem("230400",  (TObject *)230400);
		SerialSpeedComboBox->AddItem("250000",  (TObject *)250000);
		SerialSpeedComboBox->AddItem("460800",  (TObject *)460800);
		SerialSpeedComboBox->AddItem("500000",  (TObject *)500000);
		SerialSpeedComboBox->AddItem("921600",  (TObject *)921600);
		SerialSpeedComboBox->AddItem("1000000", (TObject *)1000000);
		SerialSpeedComboBox->AddItem("1843200", (TObject *)1843200);
		SerialSpeedComboBox->AddItem("2000000", (TObject *)2000000);
		SerialSpeedComboBox->AddItem("3000000", (TObject *)3000000);
		SerialSpeedComboBox->ItemIndex = SerialSpeedComboBox->Items->IndexOfObject((TObject *)DEFAULT_SERIAL_SPEED);
		SerialSpeedComboBox->OnChange = ne;
	}

	comboBoxAutoWidth(SerialPortComboBox);
	comboBoxAutoWidth(SerialSpeedComboBox);

	// ******************

	::PostMessage(this->Handle, WM_INIT_GUI, 0, 0);
}

void __fastcall TForm1::FormDestroy(TObject *Sender)
{
//
}

void __fastcall TForm1::FormClose(TObject *Sender, TCloseAction &Action)
{
	m_closing = true;

	Timer1->Enabled = false;

	if (m_thread.thread != NULL)
	{
		CThread *thread = m_thread.thread;
		m_thread.thread = NULL;
		if (!thread->FreeOnTerminate)
		{
			thread->Terminate();
			thread->WaitFor();
			delete thread;
		}
		else
			thread->Terminate();
	}

	serialDisconnect();

	saveSettings();

	if (m_console.created)
	{
		m_console.created = false;

		if (m_console.std_in)
			fclose(m_console.std_in);
		m_console.std_in = NULL;

		if (m_console.std_err)
			fclose(m_console.std_err);
		m_console.std_err = NULL;

		if (m_console.std_out)
			fclose(m_console.std_out);
		m_console.std_out = NULL;

		::FreeConsole();
	}
	
	if (m_bitmap_main)
		delete m_bitmap_main;
	m_bitmap_main = NULL;

	if (m_gdiplusToken != 0)
		Gdiplus::GdiplusShutdown(m_gdiplusToken);
	m_gdiplusToken = 0;
}

void __fastcall TForm1::WMWindowPosChanging(TWMWindowPosChanging &msg)
{
	const int thresh = 8;

	RECT work_area;
	SystemParametersInfo(SPI_GETWORKAREA, 0, &work_area, 0);

	const int dtLeft   = Screen->DesktopRect.left;
	const int dtRight  = Screen->DesktopRect.right;
	const int dtTop    = Screen->DesktopRect.top;
	const int dtBottom = Screen->DesktopRect.bottom;
	const int dtWidth  = dtRight - dtLeft;
	const int dtHeight = dtBottom - dtTop;

//	const int waLeft = work_area.left;
//	const int waTop = work_area.top;
//	const int waRight = work_area.right;
//	const int waBottom = work_area.bottom;
	const int waWidth = work_area.right - work_area.left;
	const int waHeight = work_area.bottom - work_area.top;

	int x = msg.WindowPos->x;
	int y = msg.WindowPos->y;
	int w = msg.WindowPos->cx;
	int h = msg.WindowPos->cy;

	{	// sticky screen edges
		if (std::abs((int)(x - work_area.left)) < thresh)
			x = work_area.left;			// stick left to left side
		else
		if (std::abs((int)((x + w) - work_area.right)) < thresh)
			x = work_area.right - w;	// stick right to right side

		if (std::abs((int)(y - work_area.top)) < thresh)
			y = work_area.top;			// stick top to top side
		else
		if (std::abs((int)((y + h) - work_area.bottom)) < thresh)
			y = work_area.bottom - h;	// stick bottom to bottm side

		// stick the right side to the right side of the screen if the left side is stuck to the left side of the screen
		if (x == work_area.left)
			if ((w >= (waWidth - thresh)) && (w <= (waWidth + thresh)))
				w = waWidth;

		// stick the bottom to the bottom of the screen if the top is stuck to the top of the screen
		if (y == work_area.top)
			if ((h >= (waHeight - thresh)) && (h <= (waHeight + thresh)))
				h = waHeight;
	}

	{	// limit minimum size
		if (w < Constraints->MinWidth)
			 w = Constraints->MinWidth;
		if (h < Constraints->MinHeight)
			 h = Constraints->MinHeight;
	}

	{	// limit maximum size
		if (w > Constraints->MaxWidth && Constraints->MaxWidth > Constraints->MinWidth)
			 w = Constraints->MaxWidth;
		if (h > Constraints->MaxHeight && Constraints->MaxHeight > Constraints->MinHeight)
			 h = Constraints->MaxHeight;
	}

	{	// limit maximum size
		if (w > dtWidth)
			 w = dtWidth;
		if (h > dtHeight)
			 h = dtHeight;
	}

	if (Application->MainForm && this != Application->MainForm)
	{	// stick to our main form sides
		const TRect rect = Application->MainForm->BoundsRect;

		if (std::abs((int)(x - rect.left)) < thresh)
			x = rect.left;			// stick to left to left side
		else
		if (std::abs((int)((x + w) - rect.left)) < thresh)
			x = rect.left - w;	// stick right to left side
		else
		if (std::abs((int)(x - rect.right)) < thresh)
			x = rect.right;		// stick to left to right side
		else
		if (std::abs((int)((x + w) - rect.right)) < thresh)
			x = rect.right - w;	// stick to right to right side

		if (std::abs((int)(y - rect.top)) < thresh)
			y = rect.top;			// stick top to top side
		else
		if (std::abs((int)((y + h) - rect.top)) < thresh)
			y = rect.top - h;		// stick bottom to top side
		else
		if (std::abs((int)(y - rect.bottom)) < thresh)
			y = rect.bottom;		// stick top to bottom side
		else
		if (std::abs((int)((y + h) - rect.bottom)) < thresh)
			y = rect.bottom - h;	// stick bottom to bottom side
	}

	{	// stop it completely leaving the desktop area
		if (x < (dtLeft - Width + (dtWidth / 15)))
			  x = dtLeft - Width + (dtWidth / 15);
		if (x > (dtWidth - (Screen->Width / 15)))
			  x = dtWidth - (Screen->Width / 15);
		if (y < dtTop)
			 y = dtTop;
		if (y > (dtBottom - (dtHeight / 10)))
			  y = dtBottom - (dtHeight / 10);
	}

	msg.WindowPos->x  = x;
	msg.WindowPos->y  = y;
	msg.WindowPos->cx = w;
	msg.WindowPos->cy = h;
}

void __fastcall TForm1::CMMouseEnter(TMessage &msg)
{
	TComponent *Comp = (TComponent *)msg.LParam;
	if (!Comp)
		return;

//	if (dynamic_cast<TControl *>(Comp) == NULL)
//		return;		// only interested in on screen controls

	if (dynamic_cast<TPaintBox *>(Comp) != NULL)
	{
//		TPaintBox *pb = (TPaintBox *)Comp;
//		pb->Invalidate();
	}
}

void __fastcall TForm1::CMMouseLeave(TMessage &msg)
{
	TComponent *Comp = (TComponent *)msg.LParam;
	if (!Comp)
		return;

//	if (dynamic_cast<TControl *>(Comp) == NULL)
//		return;		// only interested in on screen controls

	if (dynamic_cast<TPaintBox *>(Comp) != NULL)
	{
//		TPaintBox *pb = (TPaintBox *)Comp;
//		pb->Invalidate();
	}
}

void __fastcall TForm1::WMInitGUI(TMessage &msg)
{
	String s;

	loadSettings();

	//	BringToFront();
//	::SetForegroundWindow(Handle);

	if (Application->MainForm)
		Application->MainForm->Update();

	m_opening = false;

	if (m_console.handle)
	{
		//::ShowWindow(m_console.handle, ConsoleWindowSpeedButton->Down ? SW_SHOW : SW_HIDE);
		::ShowWindowAsync(m_console.handle, ConsoleWindowSpeedButton->Down ? SW_SHOW : SW_HIDE);

		//if (::IsWindowVisible(m_console.handle))
		//	::GetWindowRect(m_console.handle, &m_console.rect);
	}

	if (m_thread.thread == NULL)
	{	// create & start the thread
//		m_thread.thread = new CThread(&threadProcess, tpNormal, 10, true, false);
		m_thread.thread = new CThread(&threadProcess, tpNormal, 10, true, true);   // sync with main thread
	}

	// open the serial port
	SerialPortComboBoxSelect(NULL);

	Timer1->Interval = (m_thread.thread == NULL) ? 1 : 100;
	Timer1->Enabled = true;
}

void __fastcall TForm1::WMConnect(TMessage &msg)
{
//
}

void __fastcall TForm1::WMDisconnect(TMessage &msg)
{
//
}

void __fastcall TForm1::WMBreak(TMessage &msg)
{
	if (m_breaks < 10)
		printf("comm breaks %d\n", m_breaks);
}

/*
void __fastcall TForm1::addMemoLine(String s)
{
//	const int line_num = ::SendMessage(Memo1->Handle, EM_LINEFROMCHAR, -1, 0);

	bool scrolling = false;

	SCROLLINFO si;
	memset(&si, 0, sizeof(si));
	si.cbSize = sizeof(SCROLLINFO);
	si.fMask  = SIF_ALL;
	if (::GetScrollInfo(Memo1->Handle, SB_VERT, &si) == TRUE)
	{
		scrolling = (si.nPos <= (int)(si.nMax - si.nPage)) ? true : false;
		//StatusLabel->Caption =  (scrolling ? "# " : "  ") + IntToStr(Memo1->Lines->Count) + " " + IntToStr(si.nMin) + " " + IntToStr(si.nMax) + " " + IntToStr(si.nPage) + " " + IntToStr(si.nPos) + " " + IntToStr(si.nTrackPos);
	}

	if (scrolling || MemoPauseSpeedButton->Down)
	{	// the user is not at the bottom of the memo
		Memo1->Lines->BeginUpdate();
		Memo1->Lines->Add(s);
		Memo1->Lines->EndUpdate();
	}
	else
	{
		Memo1->Lines->Add(s);
	}
}
*/

void __fastcall TForm1::loadSettings()
{
	int          i;
	float        f;
	String       s;
	bool         b;

	TIniFile *ini = new TIniFile(m_ini_filename);
	if (ini == NULL)
		return;

	Top                     = ini->ReadInteger("MainForm", "Top",     Top);
	Left                    = ini->ReadInteger("MainForm", "Left",   Left);
	Width                   = ini->ReadInteger("MainForm", "Width",  Width);
	Height                  = ini->ReadInteger("MainForm", "Height", Height);

	if (m_console.handle != NULL)
	{
		ConsoleWindowSpeedButton->Down = ini->ReadBool("Console", "Show", ConsoleWindowSpeedButton->Down);
		//::ShowWindow(m_console.handle, ConsoleWindowSpeedButton->Down ? SW_SHOW : SW_HIDE);
		::ShowWindowAsync(m_console.handle, ConsoleWindowSpeedButton->Down ? SW_SHOW : SW_HIDE);

		m_console.rect.top    = ini->ReadInteger("Console", "Top",    m_console.rect.top);
		m_console.rect.left   = ini->ReadInteger("Console", "Left",   m_console.rect.left);
		m_console.rect.right  = ini->ReadInteger("Console", "Right",  m_console.rect.right);
		m_console.rect.bottom = ini->ReadInteger("Console", "Bottom", m_console.rect.bottom);
		if (ConsoleWindowSpeedButton->Down && m_console.rect.right > m_console.rect.left && m_console.rect.bottom > m_console.rect.top)
			::SetWindowPos(m_console.handle, HWND_TOP, m_console.rect.left, m_console.rect.top, m_console.rect.right - m_console.rect.left, m_console.rect.bottom - m_console.rect.top, SWP_SHOWWINDOW + SWP_ASYNCWINDOWPOS);
	}

	s = ini->ReadString("SerialPort", "Name", SerialPortComboBox->Text);
	i = SerialPortComboBox->Items->IndexOf(s);
	if (i >= 0)
		SerialPortComboBox->ItemIndex = i;

#if 1
	i = ini->ReadInteger("SerialPort", "Speed", DEFAULT_SERIAL_SPEED);
	i = SerialSpeedComboBox->Items->IndexOfObject((TObject *)i);
	if (i < 0)
		i = SerialSpeedComboBox->Items->IndexOfObject((TObject *)DEFAULT_SERIAL_SPEED);
	SerialSpeedComboBox->ItemIndex = i;
#endif

	NormaliseTrackBar->Position = ini->ReadInteger("Misc", "Normalised", NormaliseTrackBar->Position);

	HistogramSpeedButton->Down = ini->ReadBool("Misc", "Histogram", HistogramSpeedButton->Down);
	
	delete ini;
}

void __fastcall TForm1::saveSettings()
{
	String s;
	int    i;

	DeleteFile(m_ini_filename);

	TIniFile *ini = new TIniFile(m_ini_filename);
	if (ini == NULL)
		return;

	if (WindowState == wsNormal)
	{
		ini->WriteInteger("MainForm", "Top",    Top);
		ini->WriteInteger("MainForm", "Left",   Left);
		ini->WriteInteger("MainForm", "Width",  Width);
		ini->WriteInteger("MainForm", "Height", Height);
	}

	if (m_console.handle != NULL)
	{
		ini->WriteBool("Console", "Show", ConsoleWindowSpeedButton->Down);

		if (ConsoleWindowSpeedButton->Down)
			GetWindowRect(m_console.handle, &m_console.rect);
		ini->WriteInteger("Console", "Top",    m_console.rect.top);
		ini->WriteInteger("Console", "Left",   m_console.rect.left);
		ini->WriteInteger("Console", "Right",  m_console.rect.right);
		ini->WriteInteger("Console", "Bottom", m_console.rect.bottom);
	}

	ini->WriteString("SerialPort", "Name",  SerialPortComboBox->Text);
	ini->WriteInteger("SerialPort", "Speed", (int)SerialSpeedComboBox->Items->Objects[SerialSpeedComboBox->ItemIndex]);

	ini->WriteInteger("Misc", "Normalised", NormaliseTrackBar->Position);

	ini->WriteBool("Misc", "Histogram", HistogramSpeedButton->Down);

	delete ini;
}

void __fastcall TForm1::comboBoxAutoWidth(TComboBox *comboBox)
{
	if (!comboBox)
		return;

	#define COMBOBOX_HORIZONTAL_PADDING	4

	int itemsFullWidth = comboBox->Width;

	// get the max needed with of the items in dropdown state
	for (int i = 0; i < comboBox->Items->Count; i++)
	{
		int itemWidth = comboBox->Canvas->TextWidth(comboBox->Items->Strings[i]);
		itemWidth += 2 * COMBOBOX_HORIZONTAL_PADDING;
		if (itemsFullWidth < itemWidth)
			itemsFullWidth = itemWidth;
	}

	if (comboBox->DropDownCount < comboBox->Items->Count)
		itemsFullWidth += ::GetSystemMetrics(SM_CXVSCROLL);

	::SendMessage(comboBox->Handle, CB_SETDROPPEDWIDTH, itemsFullWidth, 0);
}

void __fastcall TForm1::updateSerialPortCombo()
{
	std::vector <T_SerialPortInfo> serial_port_list;

	m_serial.port.GetSerialPortList(serial_port_list);

	const TNotifyEvent ne = SerialPortComboBox->OnChange;
	SerialPortComboBox->OnChange = NULL;

	String name = SerialPortComboBox->Text.Trim();

	SerialPortComboBox->Clear();
	SerialPortComboBox->AddItem("None", (TObject *)0xffffffff);
	for (unsigned int i = 0; i < serial_port_list.size(); i++)
		SerialPortComboBox->AddItem(serial_port_list[i].name, (TObject *)i);

	const int i = SerialPortComboBox->Items->IndexOf(name);
	SerialPortComboBox->ItemIndex = (!name.IsEmpty() && i >= 0) ? i : 0;

	SerialPortComboBox->OnChange = ne;

	SerialPortComboBoxSelect(SerialPortComboBox);
}

void __fastcall TForm1::SerialPortComboBoxDropDown(TObject *Sender)
{
	updateSerialPortCombo();
}

void __fastcall TForm1::serialDisconnect()
{
	if (!m_serial.port.connected && !m_serial.client.ready)
		return;

	m_serial.client.ready = false;

	m_serial.port.rts = false;
	m_serial.port.dtr = false;
	m_serial.port.Disconnect();

	printf("serial port closed\n");
}

bool __fastcall TForm1::serialConnect()
{
	String s;

	serialDisconnect();

	m_serial.client.ready = false;

	String name = SerialPortComboBox->Text.Trim();

	if (name.IsEmpty() || name.LowerCase() == "none")
		return false;

	const int i = SerialSpeedComboBox->ItemIndex;
	if (i < 0)
	{	// error
		//printf("error: serial not selected\n");
		return false;
	}

	const int baudrate = (int)SerialSpeedComboBox->Items->Objects[i];
	if (baudrate <= 0)
	{	// error
		printf("error: serial baudrate .. %d\n", baudrate);
		return false;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(m_waveform_info); i++)
	{
		m_waveform_info[i].goertzel.re   = 0;
		m_waveform_info[i].goertzel.im   = 0;
		m_waveform_info[i].magnitude_rms = 0;
		m_waveform_info[i].phase_deg     = 0;
		memset(m_waveform_info[i].histogram, 0, sizeof(m_waveform_info[i].histogram));
	}

	m_breaks = 0;

	m_serial.port.rts      = false;
	m_serial.port.dtr      = false;
	m_serial.port.byteSize = 8;
	m_serial.port.parity   = NOPARITY;
	m_serial.port.stopBits = ONESTOPBIT;
	m_serial.port.baudRate = baudrate;

	const int res = m_serial.port.Connect(name);
	if (res != ERROR_SUCCESS)
	{
		printf("error: serial Connect() %d\n", res);
		SerialPortComboBox->ItemIndex = 0;
		return false;
	}

	m_frames = 0;
	memset(m_values, 0, sizeof(m_values));

	//	m_serial.client.name = name;

	printf("serial port opened '%s' @ %d Baud\n", m_serial.port.deviceName.c_str(), baudrate);

	m_serial.client.rx.buffer_wr = 0;
	m_serial.client.rx.timer.mark();

	#if 0
		// flush the RX
		while (m_serial.port.RxBytes(&m_serial.client.rx.buffer[0], m_serial.client.rx.buffer.size()) > 0)
			Sleep(50);
	#endif

	m_serial.client.ready = true;

	return true;
}

void __fastcall TForm1::processClient(t_client &client, CSerialPort *serial_port)
{
	if (!client.ready)
		return;

	if (!serial_port->connected)
		return;

	// check for timeout
	if (client.rx.buffer_wr > 0 && client.rx.timer.secs(false) >= 5)
	{	// no data rx'ed for 5 secs
		printf("error: serial rx line timeout\n");
		client.rx.buffer_wr = 0;
	}

	if (client.rx.buffer_wr <= 0)
		return;

#if 0
	// TEXT LINE mode

	// find the 1st LF or CR (end of text line)
	char *p = &client.rx.buffer[0];
	while (p < &client.rx.buffer[client.rx.buffer_wr] && *p >= 32)
		p++;
	if (p >= &client.rx.buffer[client.rx.buffer_wr])
	{	// no LF or CF found
		if (client.rx.buffer_wr >= (ARRAY_SIZE(client.rx.buffer) - 1))
		{
			client.rx.buffer[ARRAY_SIZE(client.rx.buffer) - 1] = '\0';
			printf("%s\n", client.rx.buffer);

			printf("error: serial rx buffer full\n");
			client.rx.buffer_wr = 0;
		}
		return;
	}

	// rx'ed a LF or CR

	client.rx.timer.mark();

	const int pos = (int)(p - &client.rx.buffer[0]);
	if (pos <= 0)
	{
		// remove spent data
		if ((pos + 1) < (int)client.rx.buffer_wr)
			memmove(&client.rx.buffer[0], &client.rx.buffer[pos + 1], client.rx.buffer_wr - pos + 1);
		client.rx.buffer_wr -= pos + 1;
		return;
	}

	// replace the LF/CR with a null-term char
	client.rx.buffer[pos] = '\0';

	// fetch rx'ed text line
	String s = String((char *)&client.rx.buffer[0]);

	// show the text line on screen
	if (!PauseSpeedButton->Down)
		if (ConsoleWindowSpeedButton->Down)
			printf("rx serial [%2d] .. %s\n", s.Length(), s.c_str());

	{	// remove spent data + any trailing control chars (cr's, lf's, tab's, spaces, null's etc)
		unsigned int k = pos + 1;
		while (k < client.rx.buffer_wr && client.rx.buffer[k] <= 32)
			k++;
		if (k < client.rx.buffer_wr)
		{
			memmove(&client.rx.buffer[0], &client.rx.buffer[k], client.rx.buffer_wr - k);
			client.rx.buffer_wr -= k;
		}
		else
			client.rx.buffer_wr = 0;
	}

	if (!s.IsEmpty())
	{
		// process the text line
		if (!PauseSpeedButton->Down)
			processRxLine(s);

/*
		//if (!client.ready || !client.serial_port.connected)
		if (!client.ready || serial_port == NULL)
			return;

		if (!s.IsEmpty())
		{	// send something back to the client

			if (serial_port)
			{
				if (serial_port->connected)
				{
					const int num_bytes_tx = serial_port->TxBytes(s);
					if (num_bytes_tx <= 0)
					{
						printf("error: serial TxData()\n");
						return;
					}

					if (ConsoleWindowSpeedButton->Down)
						printf("tx serial [%2d] .. %s", num_bytes_tx, s.c_str());
				}
			}
		}
*/

	}

#else
	// BINARY mode

	const t_packet *packet = (t_packet *)&client.rx.buffer[0];

	while (client.rx.buffer_wr >= sizeof(t_packet))
	{
		if (packet->marker != PACKET_MARKER)
		{
			// slide the data down on byte
			memmove(&client.rx.buffer[0], &client.rx.buffer[1], client.rx.buffer_wr - 1);
			client.rx.buffer_wr--;
			continue;
		}

		// found the start marker

		const uint16_t crc = CRC16_block(0, packet->data, sizeof(packet->data));
//		if (crc != packet->crc)
		if (crc != packet->crc && packet->crc != 0)  // allow the packet if the CRC is 0x0000
		{
			// slide the data down on byte
			memmove(&client.rx.buffer[0], &client.rx.buffer[1], client.rx.buffer_wr - 1);
			client.rx.buffer_wr--;

			printf("error: crc\r\n");
			continue;
		}

		// found an error-free packet
		break;
	}

	if (client.rx.buffer_wr < sizeof(t_packet))
		return;

	if (!PauseSpeedButton->Down)
	{
		// copy the rx'ed data into our local array
		memcpy(&m_values[0], packet->data, sizeof(m_values));

		bool except = false;

		// compute the magnitude and phase of each waveform
		for (unsigned int i = 0; i < ARRAY_SIZE(m_values) && !except; i++)
		{
			const unsigned int len = ARRAY_SIZE(m_values[i]);

			float avg_sum = 0;
			float sum = 0;

			memset(m_waveform_info[i].histogram, 0, sizeof(m_waveform_info[i].histogram));
			const unsigned int histo_len = ARRAY_SIZE(m_waveform_info[i].histogram);

			for (unsigned int k = 0; k < len && !except; k++)
			{
				float v = m_values[i][k];
				try
				{
					if (v != v || _isnan(v))
					{
						except = true;
						printf("NAN [%f]\r\n", v);
					}
					else
					{
						v = (v < -3000.0f) ? -3000.0f : (v > 3000.0f) ? 3000.0f : v;
						m_values[i][k] = v;

						avg_sum += v;
						sum += SQR(v);

						// waveform histogram
						unsigned int s = abs((int)(v * (histo_len - 1) * (1.0f / 2500)));  // 0 to (histo_len - 1)
						s = (s > (histo_len - 1)) ? (histo_len - 1) : s;
						if (s > 0)	// ignore DC
							m_waveform_info[i].histogram[s]++;
					}
				}
				catch (Exception &e)
				{
					except = true;
					printf("exception: %s\r\n", e.Message.c_str());
					//Application->ShowException(&e);
				}
				catch (...)
				{
					except = true;
					try
					{
						//throw Exception("");
					}
					catch (Exception &e)
					{
						printf("exception: %s\r\n", e.Message.c_str());
						//Application->ShowException(&e);
					}
				}
			}

			if (!except)
			{
				goertzel_block(m_values[i], len, &m_waveform_info[i].goertzel);

				try
				{
					if (m_waveform_info[i].goertzel.re == 0)
					{
						m_waveform_info[i].phase_deg  = 0;
					}
					else
					{
						m_waveform_info[i].phase_deg  = fmod((atan2(m_waveform_info[i].goertzel.im, m_waveform_info[i].goertzel.re) * RAD_TO_DEG) + 270, 360);
						m_waveform_info[i].phase_deg = (m_waveform_info[i].phase_deg >= 180) ? m_waveform_info[i].phase_deg - 360 : m_waveform_info[i].phase_deg;
					}

					m_waveform_info[i].average       = avg_sum / len;
					m_waveform_info[i].magnitude_rms = sqrt(sum / len);
				}
				catch (Exception &e)
				{
					printf("exception: %s\r\n", e.Message.c_str());
				}
			}
		}

		if (!except)
		{
			m_frames++;

			// update the screen
			PaintBox1->Invalidate();
		}
	}

	// remove spent data
	const unsigned int size = sizeof(uint32_t) + sizeof(m_values);
	if (client.rx.buffer_wr > size)
		memmove(&client.rx.buffer[0], &client.rx.buffer[1], client.rx.buffer_wr - size);
	client.rx.buffer_wr -= size;
#endif
}

void __fastcall TForm1::processSerial()
{
	if (!m_serial.port.connected || m_opening || m_closing)
		return;	// serial is closed

	if (!m_serial.client.ready)
		return;

	// *********
	// save any rx'ed bytes from the serial port into our RX buffer

	int num_bytes = 0;
/*
	#ifdef SERIAL_OVERLAPPED
		if (m_serial.port.GetBreak())
		{
			m_breaks++;
			::PostMessage(this->Handle, WM_BREAK, 0, 0);
			return;
		}
	#endif
*/
	const int rx_space = (int)ARRAY_SIZE(m_serial.client.rx.buffer) - m_serial.client.rx.buffer_wr - 1;
	if (rx_space > 0)
	{	// we have buffer space for more RX data

		num_bytes = m_serial.port.RxBytes(&m_serial.client.rx.buffer[m_serial.client.rx.buffer_wr], rx_space);
		if (num_bytes < 0)
		{	// error
			printf("error: serial RxBytes()\n");
			m_serial.client.rx.buffer_wr = 0;
		}
		else
		if (num_bytes > 0)
		{
			m_serial.client.rx.buffer_wr += num_bytes;
			m_serial.client.rx.timer.mark();
		}
	}

	// *********

	processClient(m_serial.client, &m_serial.port);
}

void __fastcall TForm1::threadProcess()
{
	if (m_opening || m_closing)
		return;

//	CCriticalSection cs(m_thread.cs, !m_thread.thread->sync);

	processSerial();
}

void __fastcall TForm1::FormKeyDown(TObject *Sender, WORD &Key,
		TShiftState Shift)
{
	switch (Key)
	{
		case VK_ESCAPE:
			Key = 0;
			Close();
			break;
		case VK_SPACE:
//			Key = 0;
			break;
		case VK_UP:		// up arrow
//			Key = 0;
			break;
		case VK_DOWN:	// down arrow
//			Key = 0;
			break;
		case VK_LEFT:	// left arrow
//			Key = 0;
			break;
		case VK_RIGHT:	// right arrow
//			Key = 0;
			break;
		case VK_PRIOR:	// page up
//			Key = 0;
			break;
		case VK_NEXT:	// page down
//			Key = 0;
			break;
	}
}

void __fastcall TForm1::Timer1Timer(TObject *Sender)
{
	String s;

	if (m_opening || m_closing)
		return;

	if (m_thread.thread != NULL)
	{
		s = FormatDateTime(" yyyy mmm dd ddd hh:nn:ss ", Now());
		s += "loc ";
		if (StatusBar1->Panels->Items[0]->Text != s)
		{
			StatusBar1->Panels->Items[0]->Text = s;
			StatusBar1->Update();
		}
	}

	{
		s = "";
		if (m_serial.port.connected)
			s.printf(" %0.0f ", m_serial.client.rx.timer.secs(false));
		if (StatusBar1->Panels->Items[1]->Text != s)
		{
			StatusBar1->Panels->Items[1]->Text = s;
			StatusBar1->Update();
		}
	}

	if (m_thread.thread == NULL)
		threadProcess();
}

void __fastcall TForm1::SerialPortComboBoxSelect(TObject *Sender)
{
	if (m_opening || m_closing)
		return;

	serialConnect();
}

void __fastcall TForm1::SerialSpeedComboBoxSelect(TObject *Sender)
{
	SerialPortComboBoxSelect(NULL);
}

void __fastcall TForm1::StatusBar1Resize(TObject *Sender)
{
	const int panel = 2;

	int x = StatusBar1->Left;
	for (int i = 0; i < StatusBar1->Panels->Count; i++)
		if (i < panel)
			x += StatusBar1->Panels->Items[i]->Width;

	StatusBar1->Panels->Items[panel]->Width = StatusBar1->Width - x;

	RECT Rect;
	StatusBar1->Perform(SB_GETRECT, 0, (LPARAM)&Rect);
}

void __fastcall TForm1::drawArc(Graphics::TBitmap *bm, const int x, const int y, const float start_angle, const float range_angle, const float radius)
{
	const int x1 = x - radius;
	const int y1 = y - radius;
	const int x2 = x + radius;
	const int y2 = y + radius;

	const int x3 = x + (int)(sin((start_angle              ) * DEG_TO_RAD) * radius + 0.5);
	const int y3 = y - (int)(cos((start_angle              ) * DEG_TO_RAD) * radius + 0.5);

	const int x4 = x + (int)(sin((start_angle + range_angle) * DEG_TO_RAD) * radius + 0.5);
	const int y4 = y - (int)(cos((start_angle + range_angle) * DEG_TO_RAD) * radius + 0.5);

	if ((x1 == x2 && y1 == y2) || (x3 == x4 && y3 == y4))
		return;

	::SetArcDirection(bm->Canvas->Handle, (range_angle > 0) ? AD_CLOCKWISE : AD_COUNTERCLOCKWISE);
	bm->Canvas->Arc(x1, y1, x2, y2, x3, y3, x4, y4);
}

void __fastcall TForm1::drawCircle(Graphics::TBitmap *bm, const int x, const int y, const int radius)
{
	bm->Canvas->Ellipse(x - radius, y - radius, x + radius, y + radius);
}

void __fastcall TForm1::PaintBox1Paint(TObject *Sender)
{
	String s;

	const TPaintBox *pb = PaintBox1;
	if (!pb)
		return;

	if (m_bitmap_main == NULL)
	{
		m_bitmap_main = new Graphics::TBitmap();
		if (!m_bitmap_main)
		{
			pb->Canvas->Brush->Style = bsSolid;
			pb->Canvas->Brush->Color = clWhite;
			pb->Canvas->FillRect(pb->Canvas->ClipRect);
			return;
		}
		m_bitmap_main->Monochrome   = false;
		m_bitmap_main->Transparent  = false;
		m_bitmap_main->PixelFormat  = pf32bit;
		m_bitmap_main->Canvas->Font = pb->Canvas->Font;
	}

	if (m_bitmap_main->Width != pb->Width)
		m_bitmap_main->Width = pb->Width;
	if (m_bitmap_main->Height != pb->Height)
		m_bitmap_main->Height = pb->Height;

	m_bitmap_main->Canvas->Pen->Mode = pmCopy;

	m_bitmap_main->Canvas->Brush->Style = bsSolid;
	m_bitmap_main->Canvas->Brush->Color = TColor(RGB(32, 32, 32)); // pb->Color;
	m_bitmap_main->Canvas->FillRect(m_bitmap_main->Canvas->ClipRect);

	// *********************************************************

	Gdiplus::Graphics g(m_bitmap_main->Canvas->Handle);

	//g.SetSmoothingMode(Gdiplus::SmoothingModeNone);
	//g.SetSmoothingMode(Gdiplus::SmoothingModeHighSpeed);
	//g.SetSmoothingMode(Gdiplus::SmoothingModeHighQuality);
	g.SetSmoothingMode(Gdiplus::SmoothingModeAntiAlias);

	g.SetCompositingMode(Gdiplus::CompositingModeSourceOver);
	//g.SetCompositingMode(Gdiplus::CompositingModeSourceCopy);

	//g.SetCompositingQuality(Gdiplus::CompositingQualityHighSpeed);
	g.SetCompositingQuality(Gdiplus::CompositingQualityHighQuality);
//	g.SetCompositingQuality(Gdiplus::CompositingQualityGammaCorrected);
	//g.SetCompositingQuality(Gdiplus::CompositingQualityAssumeLinear);

	//g.SetPixelOffsetMode(Gdiplus::PixelOffsetModeInvalid);
	//g.SetPixelOffsetMode(Gdiplus::PixelOffsetModeDefault);
	//g.SetPixelOffsetMode(Gdiplus::PixelOffsetModeHighSpeed);
	g.SetPixelOffsetMode(Gdiplus::PixelOffsetModeHighQuality);
//	g.SetPixelOffsetMode(Gdiplus::PixelOffsetModeNone);
	//g.SetPixelOffsetMode(Gdiplus::PixelOffsetModeHalf);

 	// *********************************************************

	const int text_height = m_bitmap_main->Canvas->TextHeight("|Hq");

	const int top_margin   = 5 + text_height + 5;
	const int bot_margin   = 30;
	const int left_margin  = m_bitmap_main->Canvas->TextWidth("------------");
	const int right_margin = 10;

	{
		const unsigned int waveforms = ARRAY_SIZE(m_values);
		const unsigned int values    = ARRAY_SIZE(m_values[0]);

		const float peak_value = 2500;

		const int x_size = m_bitmap_main->Width / 2;
		const int y_size = (m_bitmap_main->Height - top_margin - bot_margin) / (waveforms / 4);

		const float x_scale = (float)(x_size - left_margin - right_margin) / (values - 1);
		const float y_scale = (float)((y_size / 2) - 10) / peak_value;

		const float dash_pattern[]  = {5, 5};
		const float dash_pattern2[] = {2, 2};

		std::vector <Gdiplus::PointF> gdi_points(values);
		std::vector <Gdiplus::PointF> gdi_points_normalize(values);
		std::vector <Gdiplus::PointF> gdi_points_histo;

		for (unsigned int i = 0; i < waveforms; i++)
		{
			const int x = x_size * (i / 4);
			const int cy = top_margin + (y_size / 2) + (y_size * ((i / 2) % 2));

			float min = m_values[i][0];
			float max = m_values[i][0];
			for (unsigned int k = 1; k < values; k++)
			{
				const float v = m_values[i][k];
				if (min > v) min = v;
				if (max < v) max = v;
			}

			const float peak           = (fabs(max) >= fabs(min)) ? fabs(max) : fabs(min);
			const float normalize_gain = (peak > 0) ? peak_value / peak : 1.0f;

			for (unsigned int k = 0; k < values; k++)
			{
				gdi_points[k].X = x + left_margin + (k * x_scale);
				gdi_points[k].Y = cy - (m_values[i][k] * y_scale);
				gdi_points_normalize[k].X = gdi_points[k].X;
				gdi_points_normalize[k].Y = cy - (m_values[i][k] * y_scale * normalize_gain);
			}

			m_bitmap_main->Canvas->Pen->Width = 1;

			if ((i & 1) == 0)
			{	// waveform background area
				const int x1 = x + left_margin;
				const int y1 = cy - (int)(peak_value * y_scale) - 4;
				const int x2 = x + x_size - right_margin;
				const int y2 = cy + (int)(peak_value * y_scale) + 4;

				m_bitmap_main->Canvas->Brush->Style = bsSolid;
				m_bitmap_main->Canvas->Brush->Color = TColor(RGB(0, 0, 0));
				m_bitmap_main->Canvas->FillRect(TRect(x1, y1, x2, y2));
			}

			if ((i & 1) == 0)
			{	// center line
				Gdiplus::Pen pen(Gdiplus::Color(255, 100, 100, 100), 1);  // ARGB
				pen.SetAlignment(Gdiplus::PenAlignmentCenter);
				pen.SetDashStyle(Gdiplus::DashStyleDash);
				g.DrawLine(&pen, x + left_margin, cy, x + x_size - right_margin, cy);
			}

			if (HistogramSpeedButton->Down && (i & 1) == 0)
			{	// histogram
				const unsigned int histo_len = ARRAY_SIZE(m_waveform_info[i].histogram);

				const float u_scale = (float)(x_size - left_margin - right_margin) / (histo_len - 1);
				const float v_scale = (float)((y_size / 2) * 4) / 255;

				gdi_points_histo.resize(0);

				Gdiplus::PointF p1;
				Gdiplus::PointF p2;

				for (unsigned int k = 0; k < histo_len; k++)
				{
					p2.X = x + left_margin + (k * u_scale);
					p2.Y = cy - (m_waveform_info[i].histogram[k] * v_scale);
					if (k == 0)
						p1 = p2;
					p1.Y = p2.Y;
					gdi_points_histo.push_back(p1);
					gdi_points_histo.push_back(p2);
					p1 = p2;
				}

				Gdiplus::Pen pen(Gdiplus::Color(80, 128, 255, 128), 1);    // ARGB
				pen.SetAlignment(Gdiplus::PenAlignmentCenter);
				//pen.SetDashStyle(Gdiplus::DashStyleDash);
				//pen.SetDashPattern(dash_pattern, ARRAY_SIZE(dash_pattern));
				g.DrawLines(&pen, &gdi_points_histo[0], gdi_points_histo.size());
			}

			if (NormaliseTrackBar->Position > 0)
			{	// normalized waveform (max amplitude)
				float alpha = (float)NormaliseTrackBar->Position / NormaliseTrackBar->Max;  // 0 to 1.0
				alpha = alpha * alpha * 255;  // 0 to 255

				if ((i & 1) == 0)
				{
					Gdiplus::Pen pen(Gdiplus::Color((unsigned int)alpha, 255, 255, 255), 1);    // ARGB
					pen.SetAlignment(Gdiplus::PenAlignmentCenter);
					//pen.SetDashStyle(Gdiplus::DashStyleDash);
					pen.SetDashPattern(dash_pattern, ARRAY_SIZE(dash_pattern));
					g.DrawLines(&pen, &gdi_points_normalize[0], gdi_points_normalize.size());
				}
				else
				{
					Gdiplus::Pen pen(Gdiplus::Color((unsigned int)alpha, 255, 255, 80), 1);    // ARGB
					pen.SetAlignment(Gdiplus::PenAlignmentCenter);
					//pen.SetDashStyle(Gdiplus::DashStyleDash);
					pen.SetDashPattern(dash_pattern, ARRAY_SIZE(dash_pattern));
					g.DrawLines(&pen, &gdi_points_normalize[0], gdi_points_normalize.size());
				}
			}

			{	// waveform
				if ((i & 1) == 0)
				{
					Gdiplus::Pen pen(Gdiplus::Color(255, 255, 255, 255), 1);  // ARGB
					pen.SetAlignment(Gdiplus::PenAlignmentCenter);
					g.DrawLines(&pen, &gdi_points[0], gdi_points.size());
				}
				else
				{
					Gdiplus::Pen pen(Gdiplus::Color(255, 255, 255, 80), 1);  // ARGB
					pen.SetAlignment(Gdiplus::PenAlignmentCenter);
					g.DrawLines(&pen, &gdi_points[0], gdi_points.size());
				}
			}

			if ((i & 1) == 0)
			{	// waveform number
				switch (i)
				{
					case 0: s = " V-LO "; break;
					case 2: s = " I-LO "; break;
					case 4: s = " V-HI "; break;
					case 6: s = " I-HI "; break;
				}
				m_bitmap_main->Canvas->Font->Color = clWhite;
				//m_bitmap_main->Canvas->Brush->Color = pb->Color;
				m_bitmap_main->Canvas->Brush->Style = bsClear;
				//m_bitmap_main->Canvas->TextOut(x, cy - (text_height / 2), s);
				m_bitmap_main->Canvas->TextOut(x + left_margin - m_bitmap_main->Canvas->TextWidth(s), cy - text_height, s);
			}

			if ((i & 1) == 0)
			{	// phase difference

				m_bitmap_main->Canvas->Font->Color = clWhite;
				//m_bitmap_main->Canvas->Brush->Color = pb->Color;
				m_bitmap_main->Canvas->Brush->Style = bsClear;

				const float pd = phase_diff(m_waveform_info[i + 0].phase_deg, m_waveform_info[i + 1].phase_deg);
				s.printf(" %0.3f\xb0 ", pd);
				m_bitmap_main->Canvas->TextOut(x + left_margin - m_bitmap_main->Canvas->TextWidth(s), cy, s);
			}

			if ((i & 1) == 0)
			{	// vertical scale

				m_bitmap_main->Canvas->Font->Color = clWhite;
				//m_bitmap_main->Canvas->Brush->Color = pb->Color;
				m_bitmap_main->Canvas->Brush->Style = bsClear;

				Gdiplus::Pen pen(Gdiplus::Color(8, 255, 255, 255), 1);    // ARGB
				pen.SetAlignment(Gdiplus::PenAlignmentCenter);
				//pen.SetDashStyle(Gdiplus::DashStyleDash);
				pen.SetDashPattern(dash_pattern2, ARRAY_SIZE(dash_pattern2));

				const unsigned int levels = 8;

				for (unsigned int i = 2; i <= levels; i++)
				{
					const int level = (peak_value * i) / levels;
					const int y1    = cy - (int)(level * y_scale);
					const int y2    = cy + (int)(level * y_scale);
					g.DrawLine(&pen, x + left_margin, y1, x + x_size - right_margin, y1);
					g.DrawLine(&pen, x + left_margin, y2, x + x_size - right_margin, y2);

					String s1, s2;
					s1.printf(" %d ", level);
					s2.printf(" %d ", -level);
					const int tx1 = x + left_margin - m_bitmap_main->Canvas->TextWidth(s1);
					const int tx2 = x + left_margin - m_bitmap_main->Canvas->TextWidth(s2);
					m_bitmap_main->Canvas->TextOut(tx1, y1 - (text_height / 2), s1);
					m_bitmap_main->Canvas->TextOut(tx2, y2 - (text_height / 2), s2);
				}
			}

/*
			if ((i & 1) == 0)
			{	// min/max text
				m_bitmap_main->Canvas->Font->Color = clYellow;
				m_bitmap_main->Canvas->Brush->Style = bsClear;
				s.printf(" %-0.1f ", max);
				m_bitmap_main->Canvas->TextOut(x + left_margin - m_bitmap_main->Canvas->TextWidth(s), cy - (y_size / 2), s);
				s.printf(" %-0.1f ", m_waveform_info[i].average);
				m_bitmap_main->Canvas->TextOut(x + left_margin - m_bitmap_main->Canvas->TextWidth(s), cy - (text_height / 2), s);
				s.printf(" %-0.1f ", min);
				m_bitmap_main->Canvas->TextOut(x + left_margin - m_bitmap_main->Canvas->TextWidth(s), cy + (y_size / 2) - text_height, s);
			}
*/
			{	// phase and magnitude text
				const int tx  = x + left_margin + 10;
				const int ty  = (i & 1) ? cy + (y_size / 2) - 10 - (text_height * 5) : cy - (y_size / 2) + 10;
				const int ty1 = ty + (text_height * 0);
				const int ty2 = ty + (text_height * 1);
				const int ty3 = ty + (text_height * 2);
				const int ty4 = ty + (text_height * 3);
				const int ty5 = ty + (text_height * 4);

				Gdiplus::SolidBrush brush(Gdiplus::Color(160, 0, 0, 0));	// ARGB

				m_bitmap_main->Canvas->Font->Color = ((i & 1) == 0) ? clWhite : clYellow;

				m_bitmap_main->Canvas->Brush->Style = bsClear;
				//m_bitmap_main->Canvas->Brush->Color = pb->Color;

				s = ((i & 1) == 0) ? "   ADC " : "   AFC ";
				g.FillRectangle(&brush, tx, ty1, m_bitmap_main->Canvas->TextWidth(s), text_height);
				m_bitmap_main->Canvas->TextOut(tx, ty1, s);

				s.printf("   avg %0.3f ", m_waveform_info[i].average);
				g.FillRectangle(&brush, tx, ty2, m_bitmap_main->Canvas->TextWidth(s), text_height);
				m_bitmap_main->Canvas->TextOut(tx, ty2, s);

				s.printf("   mag %0.3f peak ", m_waveform_info[i].magnitude_rms * sqrt(2));
				g.FillRectangle(&brush, tx, ty3, m_bitmap_main->Canvas->TextWidth(s), text_height);
				m_bitmap_main->Canvas->TextOut(tx, ty3, s);

				s.printf("   mag %0.3f rms ", m_waveform_info[i].magnitude_rms);
				g.FillRectangle(&brush, tx, ty4, m_bitmap_main->Canvas->TextWidth(s), text_height);
				m_bitmap_main->Canvas->TextOut(tx, ty4, s);

				s.printf(" phase %+0.3f\xb0 ", m_waveform_info[i].phase_deg);
				g.FillRectangle(&brush, tx, ty5, m_bitmap_main->Canvas->TextWidth(s), text_height);
				m_bitmap_main->Canvas->TextOut(tx, ty5, s);
			}
		}

		{	// top text
			s.printf(" frame %u ", m_frames);
			m_bitmap_main->Canvas->Font->Color = clYellow;
			m_bitmap_main->Canvas->Brush->Style = bsClear;
			m_bitmap_main->Canvas->TextOut(left_margin, 5, s);
		}

		{	// min/max limits
			s.printf(" +-%.0f ", peak_value);
			m_bitmap_main->Canvas->Font->Color = clYellow;
			m_bitmap_main->Canvas->Brush->Style = bsClear;
			m_bitmap_main->Canvas->TextOut(left_margin + 100, 5, s);
		}
	}

	// *********************************************************

//	pb->Canvas->Lock();
		pb->Canvas->CopyMode = cmSrcCopy;
		pb->Canvas->Draw(0, 0, m_bitmap_main);
//	pb->Canvas->Unlock();
}

void __fastcall TForm1::ConsoleWindowSpeedButtonClick(TObject *Sender)
{
	if (m_console.handle)
	{
		const bool show = ConsoleWindowSpeedButton->Down;

		//if (!show)
		if (IsWindowVisible(m_console.handle))
			::GetWindowRect(m_console.handle, &m_console.rect);

		::ShowWindow(m_console.handle, show ? SW_SHOW : SW_HIDE);
		//::ShowWindowAsync(m_console.handle, ConsoleWindowSpeedButton->Down ? SW_SHOW : SW_HIDE);

		if (::IsWindowVisible(m_console.handle))
			if (m_console.rect.right > m_console.rect.left && m_console.rect.bottom > m_console.rect.top)
				::SetWindowPos(m_console.handle, HWND_TOP, m_console.rect.left, m_console.rect.top, m_console.rect.right - m_console.rect.left, m_console.rect.bottom - m_console.rect.top, SWP_SHOWWINDOW + SWP_ASYNCWINDOWPOS);

		//if (show && m_console.rect.right > m_console.rect.left && m_console.rect.bottom > m_console.rect.top)
		//	::SetWindowPos(m_console.handle, HWND_TOPMOST, m_console.rect.left, m_console.rect.top, m_console.rect.right - m_console.rect.left, m_console.rect.bottom - m_console.rect.top, SWP_SHOWWINDOW + SWP_ASYNCWINDOWPOS);
	}
}

void __fastcall TForm1::CaptureButtonClick(TObject *Sender)
{
	// copy the entire form image into the bitmap
	TForm *window = this;

	Graphics::TBitmap *bm = new Graphics::TBitmap();
	if (bm == NULL)
		return;

	const HDC dc = ::GetWindowDC(window->Handle);
	if (dc == NULL)
	{
		delete bm;
		return;
	}

	WINDOWINFO wi;
	ZeroMemory(&wi, sizeof(wi));
	wi.cbSize = sizeof(wi);
	::GetWindowInfo(window->Handle, &wi);

	const int cx = wi.cxWindowBorders;
	const int cy = wi.cyWindowBorders;
	const int cw = window->Width  - (wi.cxWindowBorders * 2);
	const int ch = window->Height - (wi.cyWindowBorders * 2);

	bm->Monochrome           = false;
	bm->Transparent          = false;
	bm->PixelFormat          = pf24bit;
#if 0
	bm->Width                = (cw / 8) * 8; // video codes like multiples of 8
	bm->Height               = (ch / 8) * 8; //  "
#else
	bm->Width                = cw;
	bm->Height               = ch;
#endif

	//bm->Canvas->Brush->Color = clBlack;
	//bm->Canvas->FillRect(bm->Canvas->ClipRect);

	//const int oldMode = ::SetStretchBltMode(COLORONCOLOR);  // HALFTONE
	::BitBlt(bm->Canvas->Handle, 0, 0, bm->Width, bm->Height, dc, cx, cy, SRCCOPY);
	//::SetStretchBltMode(oldMode);

	::ReleaseDC(window->Handle, dc);

	try
	{
		Clipboard()->Assign(bm);		// copy the image to the clipboard
	}
	catch (Exception &exception)
	{
		Application->NormalizeTopMosts();
		Application->MessageBox("Failed to copy to clipboard", "Error", MB_ICONERROR | MB_OK | MB_DEFBUTTON2);
		//const int res = Application->MessageBox("Failed to copy to clipboard", "Error", MB_ICONERROR | MB_OK | MB_DEFBUTTON2);
		Application->RestoreTopMosts();
	}

	delete bm;
}

void __fastcall TForm1::NormaliseTrackBarChange(TObject *Sender)
{
	PaintBox1->Invalidate();
}

void __fastcall TForm1::HistogramSpeedButtonClick(TObject *Sender)
{
	PaintBox1->Invalidate();
}

