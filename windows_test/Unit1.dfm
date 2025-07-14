object Form1: TForm1
  Left = 284
  Top = 145
  Width = 800
  Height = 571
  Caption = 'Form1'
  Color = clBtnFace
  Constraints.MinHeight = 488
  Constraints.MinWidth = 800
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  KeyPreview = True
  OldCreateOrder = False
  OnClose = FormClose
  OnCreate = FormCreate
  OnDestroy = FormDestroy
  OnKeyDown = FormKeyDown
  PixelsPerInch = 96
  TextHeight = 13
  object PaintBox1: TPaintBox
    Left = 0
    Top = 45
    Width = 792
    Height = 466
    Align = alClient
    Color = clBlack
    Font.Charset = ANSI_CHARSET
    Font.Color = clYellow
    Font.Height = -13
    Font.Name = 'Consolas'
    Font.Style = []
    ParentColor = False
    ParentFont = False
    OnPaint = PaintBox1Paint
  end
  object StatusBar1: TStatusBar
    Left = 0
    Top = 511
    Width = 792
    Height = 29
    Font.Charset = ANSI_CHARSET
    Font.Color = clBtnText
    Font.Height = -13
    Font.Name = 'Consolas'
    Font.Style = [fsBold]
    Panels = <
      item
        Alignment = taCenter
        Bevel = pbNone
        Width = 230
      end
      item
        Bevel = pbNone
        Width = 100
      end
      item
        Bevel = pbNone
        Width = 100
      end
      item
        Bevel = pbNone
        Width = 50
      end>
    SimplePanel = False
    UseSystemFont = False
    OnResize = StatusBar1Resize
  end
  object Panel2: TPanel
    Left = 0
    Top = 0
    Width = 792
    Height = 45
    Align = alTop
    BevelOuter = bvNone
    TabOrder = 1
    DesignSize = (
      792
      45)
    object Label1: TLabel
      Left = 18
      Top = 17
      Width = 50
      Height = 13
      Alignment = taRightJustify
      Caption = 'Serial port '
    end
    object Label2: TLabel
      Left = 203
      Top = 17
      Width = 61
      Height = 13
      Alignment = taRightJustify
      Caption = 'Serial speed '
    end
    object ConsoleWindowSpeedButton: TSpeedButton
      Left = 715
      Top = 12
      Width = 58
      Height = 25
      Cursor = crHandPoint
      Hint = 'Show/Hide the debug console'
      AllowAllUp = True
      Anchors = [akTop, akRight]
      GroupIndex = 3
      Caption = 'Console'
      ParentShowHint = False
      ShowHint = True
      OnClick = ConsoleWindowSpeedButtonClick
    end
    object PauseSpeedButton: TSpeedButton
      Left = 388
      Top = 12
      Width = 49
      Height = 25
      Cursor = crHandPoint
      Hint = 'Run/Pause the display'
      AllowAllUp = True
      GroupIndex = 1
      Glyph.Data = {
        F6000000424DF600000000000000760000002800000010000000100000000100
        04000000000080000000F00A0000F00A00001000000010000000008080000000
        0000000000000000000000000000000000000000000000000000000000000000
        0000000000000000000000000000000000000000000000000000000000000000
        0000000000000000000000000000000000000001110000111000000111000011
        1000000111000011100000011100001110000001110000111000000111000011
        1000000111000011100000011100001110000001110000111000000111000011
        1000000000000000000000000000000000000000000000000000}
      ParentShowHint = False
      ShowHint = True
    end
    object HistogramSpeedButton: TSpeedButton
      Left = 444
      Top = 12
      Width = 69
      Height = 25
      Cursor = crHandPoint
      Hint = 'Show/hide histogram'
      AllowAllUp = True
      GroupIndex = 2
      Down = True
      Caption = 'Histogram'
      ParentShowHint = False
      ShowHint = True
      OnClick = HistogramSpeedButtonClick
    end
    object SerialPortComboBox: TComboBox
      Left = 76
      Top = 14
      Width = 105
      Height = 21
      Cursor = crHandPoint
      AutoComplete = False
      AutoDropDown = True
      Style = csDropDownList
      DropDownCount = 20
      ItemHeight = 13
      TabOrder = 0
      OnDropDown = SerialPortComboBoxDropDown
      OnSelect = SerialPortComboBoxSelect
    end
    object SerialSpeedComboBox: TComboBox
      Left = 272
      Top = 14
      Width = 105
      Height = 21
      Cursor = crHandPoint
      AutoComplete = False
      AutoDropDown = True
      Style = csDropDownList
      DropDownCount = 20
      ItemHeight = 13
      TabOrder = 1
      OnSelect = SerialSpeedComboBoxSelect
    end
    object CaptureButton: TButton
      Left = 634
      Top = 12
      Width = 75
      Height = 25
      Cursor = crHandPoint
      Hint = 'Copy the form image'
      Anchors = [akTop, akRight]
      Caption = 'Capture'
      ParentShowHint = False
      ShowHint = True
      TabOrder = 2
      OnClick = CaptureButtonClick
    end
    object NormaliseTrackBar: TTrackBar
      Left = 516
      Top = 12
      Width = 113
      Height = 25
      Cursor = crHandPoint
      Hint = 'Normalised waveform brightness'
      Max = 30
      Orientation = trHorizontal
      ParentShowHint = False
      Frequency = 1
      Position = 0
      SelEnd = 0
      SelStart = 0
      ShowHint = True
      TabOrder = 3
      TickMarks = tmBoth
      TickStyle = tsNone
      OnChange = NormaliseTrackBarChange
    end
  end
  object OpenDialog1: TOpenDialog
    DefaultExt = '.bin'
    FileName = 'my_config.bin'
    Filter = 
      'All files|*.*|Supported files *.bin *.raw|*.bin;*.raw|BIN files|' +
      '*.bin|RAW files|*.raw'
    FilterIndex = 2
    Options = [ofReadOnly, ofPathMustExist, ofFileMustExist, ofEnableSizing]
    Left = 84
    Top = 64
  end
  object SaveDialog1: TSaveDialog
    DefaultExt = '.bin'
    FileName = 'my_config.bin'
    Filter = 
      'All files|*.*|Supported files *.bin *.raw|*.bin;*.raw|BIN files|' +
      '*.bin|RAW files|*.raw'
    FilterIndex = 2
    Options = [ofOverwritePrompt, ofHideReadOnly, ofPathMustExist, ofEnableSizing]
    Left = 140
    Top = 64
  end
  object Timer1: TTimer
    Enabled = False
    Interval = 100
    OnTimer = Timer1Timer
    Left = 32
    Top = 60
  end
end
