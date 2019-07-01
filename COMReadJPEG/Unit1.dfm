object Form1: TForm1
  Left = 482
  Top = 141
  BorderStyle = bsSingle
  Caption = 'Form1'
  ClientHeight = 683
  ClientWidth = 830
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  OldCreateOrder = False
  PixelsPerInch = 96
  TextHeight = 13
  object Image1: TImage
    Left = 16
    Top = 80
    Width = 801
    Height = 601
  end
  object Image2: TImage
    Left = 16
    Top = 80
    Width = 801
    Height = 601
    Visible = False
  end
  object Label7: TLabel
    Left = 420
    Top = 12
    Width = 63
    Height = 13
    Caption = 'quality (4-63):'
  end
  object Label8: TLabel
    Left = 420
    Top = 36
    Width = 53
    Height = 13
    Caption = 'Resolution:'
  end
  object Label9: TLabel
    Left = 420
    Top = 60
    Width = 61
    Height = 13
    Caption = 'Update (fps):'
  end
  object Label10: TLabel
    Left = 16
    Top = 8
    Width = 45
    Height = 13
    Caption = 'COM port'
  end
  object CheckBox1: TCheckBox
    Left = 596
    Top = 60
    Width = 57
    Height = 17
    Caption = 'ZOOM'
    TabOrder = 0
  end
  object GroupBox1: TGroupBox
    Left = 204
    Top = 12
    Width = 205
    Height = 65
    Caption = 'Info about image'
    TabOrder = 1
    object Label1: TLabel
      Left = 108
      Top = 16
      Width = 34
      Height = 13
      Caption = '0/0 fps'
    end
    object Label2: TLabel
      Left = 108
      Top = 32
      Width = 26
      Height = 13
      Caption = '0 bps'
    end
    object Label3: TLabel
      Left = 108
      Top = 48
      Width = 29
      Height = 13
      Caption = '0 byte'
    end
    object Label4: TLabel
      Left = 8
      Top = 16
      Width = 38
      Height = 13
      Caption = 'Update:'
    end
    object Label5: TLabel
      Left = 8
      Top = 32
      Width = 58
      Height = 13
      Caption = 'Speed rade:'
    end
    object Label6: TLabel
      Left = 8
      Top = 48
      Width = 63
      Height = 13
      Caption = 'Size of image'
    end
  end
  object CheckBox2: TCheckBox
    Left = 596
    Top = 40
    Width = 61
    Height = 17
    Caption = 'BLACK'
    TabOrder = 2
  end
  object Button4: TButton
    Left = 592
    Top = 8
    Width = 77
    Height = 29
    Caption = 'Set'
    Enabled = False
    TabOrder = 3
    OnClick = Button4Click
  end
  object Button5: TButton
    Left = 692
    Top = 8
    Width = 101
    Height = 29
    Caption = 'Save Image'
    Enabled = False
    TabOrder = 4
    OnClick = Button5Click
  end
  object Button6: TButton
    Left = 692
    Top = 52
    Width = 101
    Height = 25
    Caption = 'Video'
    Enabled = False
    TabOrder = 5
    OnClick = Button6Click
  end
  object ComboBox2: TComboBox
    Left = 512
    Top = 32
    Width = 77
    Height = 21
    ItemHeight = 13
    TabOrder = 6
    Text = '320x240'
    Items.Strings = (
      '320x240'
      '640x480'
      '800x600'
      '1024x768'
      '1280x1024'
      '1600x1200')
  end
  object Edit1: TEdit
    Left = 512
    Top = 8
    Width = 77
    Height = 21
    TabOrder = 7
    Text = '40'
  end
  object Edit2: TEdit
    Left = 512
    Top = 56
    Width = 77
    Height = 21
    TabOrder = 8
    Text = '20'
  end
  object Button1: TButton
    Left = 104
    Top = 20
    Width = 81
    Height = 25
    Caption = 'Connect'
    TabOrder = 9
    OnClick = Button1Click
  end
  object Button2: TButton
    Left = 104
    Top = 52
    Width = 81
    Height = 25
    Caption = 'Disconnect'
    TabOrder = 10
    OnClick = Button2Click
  end
  object Button3: TButton
    Left = 16
    Top = 52
    Width = 75
    Height = 25
    Caption = 'Find Port'
    TabOrder = 11
    OnClick = Button3Click
  end
  object ComboBox1: TComboBox
    Left = 16
    Top = 24
    Width = 73
    Height = 21
    ItemHeight = 13
    TabOrder = 12
    Text = 'COM1'
  end
  object Memo1: TMemo
    Left = 872
    Top = 480
    Width = 185
    Height = 89
    Lines.Strings = (
      'Memo1')
    TabOrder = 13
    Visible = False
  end
  object Timer1: TTimer
    OnTimer = Timer1Timer
    Left = 444
    Top = 84
  end
  object Timer2: TTimer
    Enabled = False
    Interval = 25
    OnTimer = Timer2Timer
    Left = 612
    Top = 88
  end
  object Timer3: TTimer
    Interval = 10000
    OnTimer = Timer3Timer
    Left = 680
    Top = 88
  end
end
