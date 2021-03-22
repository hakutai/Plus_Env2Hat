/*
    note: need add library Adafruit_BMP280 from library manage
    Github: https://github.com/adafruit/Adafruit_BMP280_Library
*/

/**
 * @file Plus_Env2Hat.ino
 * 
 * @note 色指定が演算式になっているから色決定後に定数にすること
 * @note M5.Lcdに直接書き込んだ後は「M5.Lcd.textWidth」が正しい値を返してこないポイ（値が倍になるみたい、追試必要）
 *         すべてダブルバッファーを使用すること。deepsleep解除後は正常になるので回避方法はあるかも？
 * @note font1-size2は９ｘ１２ピクセルで作る
 * 
 * 
 * @version
 *   2021/03/04 1.00   履歴にＷBGT指数の表示 
 *   2021/03/05 1.01   外部電源中にディープスリープから復帰した場合は自動でディープスリープへ
 *   2021/03/22 1.02   外部電源充電中に指定電圧（BATTERMAX）以下時に確認用にLEDを点灯
 * 
*/

#include <M5StickCPlus.h>
#include <math.h>
#include "SHT3X.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "bmm150.h"
#include "bmm150_defs.h"
#include "ImgComp.h"

/*---------------------------------------------------------
 * 各センサー用変数
 */
SHT3X                           sht3x;                              // 温湿度センサー
Adafruit_BMP280                 bmp280;                             // 気圧センサー
BMM150                          bmm150 = BMM150();                  // 地磁気センサー
RTC_DATA_ATTR bmm150_mag_data   bmm150Offset = { 0, 0, 0 };         // 地磁気センサの補正値

/*---------------------------------------------------------
 * 大域変数　
 *          処理速度を上げるためにローカル変数や関数呼び出しにスタックを極力使わない
 */
        // センサー情報
float       temperature   = 0.0;      // 温度
float       humidity      = 0.0;      // 湿度
float       pressure      = 0.0;      // 気圧
float       altitude      = 0.0;      // 高度
int         wbgtIndex     = 0.0;      // 暑さ指数（Wet Bulb Globe Temperature）
int         discomfortIndex = 0.0;    // 不快指数（discomfort index）
float       temperature280;           // BMP280の温度

RTC_DateTypeDef           rtcDate;    // 年月日
RTC_TimeTypeDef           rtcTime;    // 日時秒

        // 動作設定用
#define WAKEUP_TIME       SLEEP_MIN(30)         // ディープ休眠時間
#define DEMOTIME          5000                  // デモモードの画面切り替え時間（5000ミリ秒）

RTC_DATA_ATTR uint8_t     scrnMode      = 0;    // LCD表示内容　(0:温度・湿度,1:気圧・高度,2:方位,3:履歴）
RTC_DATA_ATTR uint8_t     lcdDirection  = 1;    // LCDの向き　 1 or 3  
RTC_DATA_ATTR uint8_t     lcdBrightness = 9;    // LCDの明るさ 7 to 15
RTC_DATA_ATTR uint32_t    defaultPowerOffTime = 20000;      // スリープ時間
RTC_DATA_ATTR uint8_t     resumeOn      = false;  // レジューム（スリープ時の表示に戻る）
uint32_t                  demoMode      = 0;    // デモモード　0:オフ,>0:オン（切替までのミリ秒）

uint32_t    powerOffTime = defaultPowerOffTime;    // ディープスリープへ移行する時間

#define     LED_PIN       GPIO_NUM_10           // 付属LEDのGPIOの番号
#define     BATTERYMAX    4.15                  // 満充電判断の閾値   個体により目安にする電圧が違う、個々に値を決めること 
 

        // 画面用　ダブルバッファー他
int16_t       scrnWidth,scrnHeight;   // スクリーン（LCD)縦横
TFT_eSprite   lcdDblBuf = TFT_eSprite(&M5.Lcd);   // ダブルバッファー
TFT_eSprite   copsBuf   = TFT_eSprite(&M5.Lcd);   // コンパス画像

uint32_t      update_time   = 0;                  // ＬCD書き換え間隔管理

/*
 * 気圧変動の確認用にスローメモリーへ保存する構造体、日付・気温・湿度・気圧を３時間ごとに保存
 */
#define PRESSURE_UC       0             // 24h前と気圧変化無し
#define PRESSURE_UP       1             //       5hPa以上上がる
#define PRESSURE_DN       2             //       5hPa以上下がる
typedef struct PressArray_ {            //  RTCメモリへ保存構造体
  int      day;                         //    日
  uint8_t  pressureUpDn;                //    気圧変化　上記３つの定数を入れる
  int      pressure;                    //    気圧
  int      temperature;                 //    温度
  int      humidity;                    //    湿度
} PressArray;

#define CLEAR_PRESSARRAY {0,0,0,0,0}    // 構造体初期化値
RTC_DATA_ATTR PressArray presAry[8] = { CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY,
                                        CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY, CLEAR_PRESSARRAY };

/*
 * 気圧関連
 */
#define   DEFAULT_SEALEVEL        1013.25     // 標準気圧
#define   SEALEVEL_TEMPERATURE    15          // 海面温度
RTC_DATA_ATTR float     seaLevelPressure = DEFAULT_SEALEVEL;      // 設定海面気圧
uint16_t  kAltitude = 0;                      // 標高校正用カウンタ

/*
 * 電源監視用
 */
uint8_t     extPW = false;                    /* 電源使用 true:外部電源 / false:内部電源 */
double      pwVolt;                           // Power Voltage
double      pwCurt;                           // Power Current

/*===========================================================================
 * 汎用変数
 */
int16_t     i,j;                 // カウンター
char        tmpStr[40];          // 汎用文字列

/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * 各種　一行計算式
 * 
 */
//  熱中症指数、不快指数の算出
#define CalcWBGTIndex(temp,humi)         (temp * (0.62 + humi * 0.005))
#define CalcdiscomfortIndex(temp,humi)   (0.81 * temp + 0.01 * humi * (0.99 * temp - 14.3) + 46.3)
//  標高ー＞気圧変換
#define   AltToPres(altitude)          (pressure*pow((1-0.0065*altitude/(SEALEVEL_TEMPERATURE+273.2)),5.254))
// RGB565 24bit -> 16bit 色決定後に定数に直すこと
#define ToRGB565(r,g,b) ((r >> 3) << 11 | (g >> 2) << 5 | (b >> 3))

/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * M5StickC-PlusのSetSleepはバグ有り　正常動作のSetSleep 2021/02/20
 * 
 */
void xSetSleep(void)          //version M5stick-C plus
{
  M5.Axp.Write1Byte(0x31, M5.Axp.Read8bit(0x31) | ( 1 << 3)); // short press to wake up
  M5.Axp.Write1Byte(0x90, M5.Axp.Read8bit(0x90) | 0x07); // GPIO1 "low noise" otherwise it goes to download mode when waking up
  M5.Axp.Write1Byte(0x12, M5.Axp.Read8bit(0x12) & ~(1<<1) & ~(1<<2) & ~(1<<3)); // Disable all outputs but DCDC1 (DCDC3 = NC, LDO2 = LCD-BL, LDO3 = LCD-Logic)
}
uint8_t           wakeUpCause;      // 起動理由 esp_sleep_get_wakeup_cause()の返り値


/*＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 *＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
 * SETUP
 */
void setup() {

  // put your setup code here, to run once:
  M5.begin();
  Wire.begin(0,26,100000);
  M5.Lcd.setRotation(lcdDirection);
  M5.Lcd.fillScreen(BLACK);
  M5.Axp.ScreenBreath(lcdBrightness);              // LCDの明るさ　９
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,HIGH);
  scrnWidth  = M5.Lcd.width();
  scrnHeight = M5.Lcd.height();

    // LCDのダブルバッファーの用意
  lcdDblBuf.createSprite(M5.Lcd.width(),M5.Lcd.height());
  lcdDblBuf.setSwapBytes(false);
        // スプライトコンパス
  copsBuf.createSprite(imgCompWidth,imgCompHeight);
  copsBuf.setSwapBytes(false);
  copsBuf.pushImage(0, 0, imgCompWidth, imgCompHeight, imgComp);
  
  pinMode(M5_BUTTON_HOME, INPUT);
  setCpuFrequencyMhz(20);             // CPUを20MHzで駆動

  if(bmm150.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while(1);
  } else {
    Serial.println("Initialize done!");
  }
  if (!bmp280.begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
  }
    /* Default settings from datasheet. */
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1000); /* Standby time. */

  if (!resumeOn) scrnMode = 0;        // スクリーンの復帰
  if (demoMode != 0) demoMode = millis() + DEMOTIME;
  
  /*　電源ソースを調べる */
  pwVolt = M5.Axp.GetVBusVoltage();
  if (pwVolt > 3.3) extPW = true;

  /* ディープスリープ設定（ボタンA押下で起床 */
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, 0);  

  /* 起動理由の格納 */
  wakeUpCause = esp_sleep_get_wakeup_cause();

}

/*******************************************************
 *  
 */
void BMM150_Calibrate();
enum ScrnMode { TEMPHUM, PRESVOLT, COMPASS, LIST_PRES };
void Env2Menu();
void DispScrnMode0();             // 温度・湿度　デカ表示
void DispScrnMode1();             // 温度・湿度、気圧・標高、電圧・電流
void DispScrnMode2();             // コンパスの予定 
void DispScrnMode3();             // 気圧履歴

/*******************************************************
 *  LOOP
 */

void loop() {

  M5.update();
  if (M5.BtnA.wasPressed()) {
    scrnMode = (scrnMode + 1) & 0x03;
    lcdDblBuf.fillSprite(BLACK);
    update_time = 0;
    powerOffTime = defaultPowerOffTime + millis();
  }
  if (scrnMode == TEMPHUM) {
    if (M5.BtnB.isPressed()) {
      Env2Menu();
      powerOffTime = defaultPowerOffTime + millis();
    }    
  }
  if (demoMode > 0) {
    if (millis() > demoMode) {
      scrnMode = (scrnMode + 1) & 0x03;
      demoMode = millis() + DEMOTIME;    
    }
  }
  
  if (scrnMode == PRESVOLT) {
    if (M5.BtnB.isPressed()) {              // 標高の設定
      Serial.printf("Btn B press  %d\r\n",kAltitude);
      if (kAltitude <= 1) seaLevelPressure = DEFAULT_SEALEVEL;
      else                seaLevelPressure = AltToPres(-100 * (kAltitude / 2 - 1));     // 再描画間隔が短いと操作性悪い、２で割って0.5間隔で更新
      Serial.printf("Btn B press  %d  %lf\r\n",kAltitude,seaLevelPressure);
      kAltitude++;
      powerOffTime = UINT32_MAX;
    } else {
      if (kAltitude > 1) powerOffTime = defaultPowerOffTime + millis();
      kAltitude = 0;
    }
  }
  if (scrnMode == COMPASS) {
    if (M5.BtnB.isPressed()) {        // キャリブレーション
      lcdDblBuf.fillSprite(BLACK);
      lcdDblBuf.setTextSize(3);
      lcdDblBuf.setCursor(20,20, 4);
      lcdDblBuf.setTextColor(TFT_RED,TFT_WHITE);
      lcdDblBuf.printf("CAL");
      lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
      BMM150_Calibrate();                    // 釦Bを離すまで帰ってこない
      lcdDblBuf.fillScreen(BLACK);
      lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
      powerOffTime = defaultPowerOffTime + millis();
    }
  }
  
  /*--- 電源ソースを調べる ---*/
  pwVolt = M5.Axp.GetVBusVoltage();
  if (pwVolt > 3.3) extPW = true;
  else              extPW = false;

  if (millis() > update_time) {
    update_time = millis() + 500;          // 500ms間隔で再描画
    
    if(sht3x.get()==0){                     // 温度・湿度の取得
      temperature = sht3x.cTemp;
      humidity = sht3x.humidity;
      wbgtIndex = (int)CalcWBGTIndex(temperature,humidity);
      discomfortIndex = (int)CalcdiscomfortIndex(temperature,humidity);
    }

    pressure       = bmp280.readPressure() / 100;   //Pa -> hPa
    altitude       = bmp280.readAltitude(seaLevelPressure);
    temperature280 = bmp280.readTemperature();

    M5.Rtc.GetData(&rtcDate);
    M5.Rtc.GetTime(&rtcTime);

    if (int(wbgtIndex) >= 31) {     // 31
      M5.Lcd.fillScreen(BLACK);
      lcdDblBuf.pushSprite(0,0); 
      M5.Beep.tone(2000);
    } else {
      M5.Beep.mute();
    }

    M5.Lcd.startWrite();
    switch (scrnMode) {
      case TEMPHUM: 
              DispScrnMode0();                // 温度・湿度表示
              break;
      case PRESVOLT:                          // 温湿度・気圧標高・電圧流
              DispScrnMode1();
              break;
      case COMPASS:                           // 方位
              DispScrnMode2();              
              break;
      case LIST_PRES: 
              DispScrnMode3();                //　気圧履歴
              break;
      default:
              DispScrnMode0();
    }

    lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
    M5.Lcd.endWrite();
    

    /*--- 気圧をスローメモリーに保存 0,3,6,9,12,15,18,21時の３時間ごとに保存 ---*/
    if (rtcTime.Hours % 3 == 0) {  
      i = int(rtcTime.Hours / 3);
      if (presAry[i].day != rtcDate.Date) {
        if (presAry[i].pressure != 0) {
          j = (int)pressure - presAry[i].pressure;
          if (j < -5)     presAry[i].pressureUpDn = PRESSURE_DN;
          else if (j > 5) presAry[i].pressureUpDn = PRESSURE_UP;
          else            presAry[i].pressureUpDn = PRESSURE_UC;
          Serial.printf("UpDn %d %d\r\n",j,presAry[i].pressureUpDn);
        }
        presAry[i].day          = rtcDate.Date;
        presAry[i].temperature  = (int)temperature;
        presAry[i].humidity     = (int)humidity;
        presAry[i].pressure     = (int)pressure;
      }
    }
        /*--- 充電中はLEDを点灯 外部電源＆手動で稼働している ---*/
    if (extPW && (wakeUpCause != ESP_SLEEP_WAKEUP_TIMER)) {  
      pwVolt = M5.Axp.GetBatVoltage();
      Serial.printf("Battery %f(%f)\r\n",pwVolt,BATTERYMAX);
      if (pwVolt < BATTERYMAX) digitalWrite(LED_PIN,LOW);
      else                     digitalWrite(LED_PIN,HIGH);
    } else {
      digitalWrite(LED_PIN,HIGH);
    }
  /*--- DeepSleep の設定 
          バッテリーまたはdeepsleepから起動した場合は再びdeepsleepへ
  */
    if (!extPW || (wakeUpCause == ESP_SLEEP_WAKEUP_TIMER)) {             // 充電池駆動
      if   (millis() > powerOffTime) {
          //---ここからは AXP192::DeepSleep(uint64_t time_in_us)の必要部分の抜き出し
         xSetSleep();
         esp_sleep_enable_timer_wakeup(WAKEUP_TIME);      // 30分スリープ
         esp_deep_sleep(WAKEUP_TIME);
          //--- ここまで
      }
    }
  }

  delay(100);
}
/*=====================================================================
 * Functions
 */
 void BMM150_Calibrate()
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;

  bmm150.read_mag_data();  
  value_x_min = bmm150.raw_mag_data.raw_datax;
  value_x_max = bmm150.raw_mag_data.raw_datax;
  value_y_min = bmm150.raw_mag_data.raw_datay;
  value_y_max = bmm150.raw_mag_data.raw_datay;
  delay(100);

  
  while(M5.BtnB.isPressed())
  {
    M5.update();

    bmm150.read_mag_data();
    
    /* Update x-Axis max/min value */
    if(value_x_min > bmm150.raw_mag_data.raw_datax)      value_x_min = bmm150.raw_mag_data.raw_datax;
    else if(value_x_max < bmm150.raw_mag_data.raw_datax) value_x_max = bmm150.raw_mag_data.raw_datax;

    /* Update y-Axis max/min value */
    if(value_y_min > bmm150.raw_mag_data.raw_datay)      value_y_min = bmm150.raw_mag_data.raw_datay;
    else if(value_y_max < bmm150.raw_mag_data.raw_datay) value_y_max = bmm150.raw_mag_data.raw_datay;

    delay(200);
  }

  bmm150Offset.x = value_x_min + (value_x_max - value_x_min)/2;
  bmm150Offset.y = value_y_min + (value_y_max - value_y_min)/2;
}

/*=====================================================================================================
 * メニュー表示
 * 　　スクリーンの向き
 * 　　スクリーンの明るさ
 * 　　スリープへの時間
 * 　　日時
 *  設定で速度関係ないからローカル変数を使用
 */
// メニュー表示
void Env2Menu() {
  uint16_t      idxMenu = 0;
  uint16_t      btnAwasPressed;   //単純に関数嫌だから呼ぶの
  uint16_t      menuExit = false;
                               // 1  2  3  4  5  6  7  8  9 10 11 12
  uint16_t      monthDay[12] = { 31,29,31,30,31,30,31,31,30,31,30,31 };
  uint32_t      autoExitTime;

  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextFont(1);
  lcdDblBuf.fillSprite(BLACK);

  M5.Rtc.GetData(&rtcDate);
  M5.Rtc.GetTime(&rtcTime);

  autoExitTime = defaultPowerOffTime + millis();

  do {
    lcdDblBuf.setTextColor(TFT_WHITE, TFT_BLACK);
      //-- LCDの向き
    lcdDblBuf.setCursor(0,0);     lcdDblBuf.printf("Scrn Direction ");
    lcdDblBuf.setCursor(176,0);   
    if (lcdDirection == 1)     lcdDblBuf.printf(" LEFT");
    else                       lcdDblBuf.printf("RIGHT");
      //-- LCDの明るさ
    lcdDblBuf.setCursor(0,20);    lcdDblBuf.printf("Scrn Brightness ");
    lcdDblBuf.setCursor(210,20);  lcdDblBuf.printf("%2d",lcdBrightness);
      //-- スリープ時間
    lcdDblBuf.setCursor(0,40);    lcdDblBuf.printf("Sleep Time ");
    lcdDblBuf.setCursor(210,40);  lcdDblBuf.printf("%2d",defaultPowerOffTime / 1000);
      //-- レジューム（スクリーンの復帰）
    lcdDblBuf.setCursor(0,60);    lcdDblBuf.printf("Resume ");
    lcdDblBuf.setCursor(200,60);  
    if (resumeOn) lcdDblBuf.printf(" ON");
    else          lcdDblBuf.printf("OFF");
      //-- 時刻
    lcdDblBuf.setCursor(0,80);    lcdDblBuf.printf("DT ");
    lcdDblBuf.setCursor(40,80);   
      lcdDblBuf.printf("%4d/%02d/%02d %02d:%02d",
                          rtcDate.Year,rtcDate.Month,rtcDate.Date,rtcTime.Hours,rtcTime.Minutes);
      //-- デモモード
    lcdDblBuf.setCursor(0,100);    lcdDblBuf.printf("DemoMode");
    lcdDblBuf.setCursor(200,100);
    if (demoMode) lcdDblBuf.printf(" ON");
    else          lcdDblBuf.printf("OFF");
      //-- RETURN
    lcdDblBuf.setCursor(0,120);   lcdDblBuf.printf("RETURN");

    M5.update();
    if (btnAwasPressed = M5.BtnA.wasPressed()) autoExitTime = defaultPowerOffTime + millis();
    if (M5.BtnB.wasPressed()) {
      if (++idxMenu > 10) idxMenu = 0;
      autoExitTime = defaultPowerOffTime + millis();
    }

    lcdDblBuf.setTextColor(TFT_BLACK, TFT_CYAN);
    switch (idxMenu) {
      case 0:     //スクリーンの向き
                lcdDblBuf.setCursor(176,0);
                if (btnAwasPressed) {
                  lcdDirection == 1 ? lcdDirection = 3 : lcdDirection = 1;
                  lcdDblBuf.fillSprite(BLACK);
                  M5.Lcd.setRotation(lcdDirection);
                }
                if (lcdDirection == 1)  lcdDblBuf.printf(" LEFT");
                else                    lcdDblBuf.printf("RIGHT");
                break;
      case 1:     //スクリーンの明るさ
                if (btnAwasPressed) {
                  if (++lcdBrightness > 15) lcdBrightness = 7;
                  M5.Axp.ScreenBreath(lcdBrightness);              // LCDの明るさ　９
                }
                lcdDblBuf.setCursor(210,20);  lcdDblBuf.printf("%2d",lcdBrightness);
                break;
      case 2:     //スリープ時間
                if (btnAwasPressed) {
                  defaultPowerOffTime += 10000;
                  if (defaultPowerOffTime > 60000) defaultPowerOffTime = 10000;
                }
                lcdDblBuf.setCursor(210,40);  lcdDblBuf.printf("%2d",defaultPowerOffTime / 1000);
                break;
      case 3:     //レジューム
                if (btnAwasPressed) {
                  resumeOn = !resumeOn;
                }
                lcdDblBuf.setCursor(200,60);  
                if (resumeOn) lcdDblBuf.printf(" ON");
                else          lcdDblBuf.printf("OFF");
                break;
      case 4:     //年
                if (btnAwasPressed) {
                  if (++rtcDate.Year > 2030) rtcDate.Year = 2021;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40,80);   
                lcdDblBuf.printf("%4d",rtcDate.Year);
                break;
      case 5:     //月
                if (btnAwasPressed) {
                  if (++rtcDate.Month > 12) rtcDate.Month = 1;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40 + 12 * 5,80);   
                lcdDblBuf.printf("%02d",rtcDate.Month);
                break;
      case 6:     //日
                if (btnAwasPressed) {
                  if (++rtcDate.Date > monthDay[rtcDate.Month-1]) rtcDate.Date = 1;
                  M5.Rtc.SetData(&rtcDate);
                }
                lcdDblBuf.setCursor(40 + 12 * 8,80);   
                lcdDblBuf.printf("%02d",rtcDate.Date);
                break;
      case 7:     //時
                if (btnAwasPressed) {
                  if (++rtcTime.Hours > 23) rtcTime.Hours = 0;
                  M5.Rtc.SetTime(&rtcTime);
                }
                lcdDblBuf.setCursor(40 + 12 * 11,80);   
                lcdDblBuf.printf("%02d",rtcTime.Hours);
                break;
      case 8:     //分
                if (btnAwasPressed) {
                  if (++rtcTime.Minutes > 59) rtcTime.Minutes = 0;
                  M5.Rtc.SetTime(&rtcTime);
                }
                lcdDblBuf.setCursor(40 + 12 * 14,80);   
                lcdDblBuf.printf("%02d",rtcTime.Minutes);
                break;
      case 9:     //デモモード
                if (btnAwasPressed) {
                  if (demoMode != 0) demoMode = 0;
                  else               demoMode = millis() + DEMOTIME;
                }
                lcdDblBuf.setCursor(200,100);  
                if (demoMode != 0) lcdDblBuf.printf(" ON");
                else               lcdDblBuf.printf("OFF");
                break;
      default:
                lcdDblBuf.setCursor(0,120);   lcdDblBuf.printf("RETURN");
                menuExit = false;
                if (btnAwasPressed) menuExit = true;
                break;
    }
    lcdDblBuf.pushSprite(0,0);                //　ダブルバッファーLCDに書き込み
                
    if (millis() > autoExitTime) menuExit = true;


    delay(100);
  } while (!menuExit);
}
/*==================================================================
 * 温度・湿度　デカ表示
 */
void DispScrnMode0() {
  // 背景
  if (wbgtIndex >= 31)      lcdDblBuf.fillSprite(ToRGB565(192,0,0));
  else if (wbgtIndex >= 28) lcdDblBuf.fillSprite(TFT_OLIVE);
  else                      lcdDblBuf.fillSprite(TFT_DARKCYAN);
  // 気温
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setCursor(5+5,1+6,4);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.printf("%.1f ",temperature);
  lcdDblBuf.setCursor(5,1);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.printf("%.1f ",temperature);
  lcdDblBuf.setTextSize(1);  lcdDblBuf.setCursor(160,3); lcdDblBuf.printf("C");

  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setCursor(165+3,23+4,4);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.printf("%2d",wbgtIndex);
  lcdDblBuf.setCursor(165,23,4);
  if (wbgtIndex < 25)      lcdDblBuf.setTextColor(TFT_CYAN);
  else if (wbgtIndex < 28) lcdDblBuf.setTextColor(TFT_GREENYELLOW);
  else if (wbgtIndex < 31) lcdDblBuf.setTextColor(TFT_YELLOW);
  else             lcdDblBuf.setTextColor(TFT_RED);
  lcdDblBuf.printf("%2d",wbgtIndex);
    // 暑さ指数上昇
  i = int((rtcTime.Hours - 3) / 3);
  if (i < 0) i = 7;
  if (presAry[i].temperature != 0) {
    j = CalcWBGTIndex(presAry[i].temperature,presAry[i].humidity);
    lcdDblBuf.setCursor(165,23,4);
    lcdDblBuf.setTextColor(TFT_RED);
    if (j - wbgtIndex < 0)  lcdDblBuf.printf("     ^");   // 前の時間帯より指数上昇
    else                    lcdDblBuf.printf("      ");
  }
  
  // 湿度
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setCursor(5+5,67+6);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.printf("%.1f ",humidity);
  lcdDblBuf.setCursor(5,67);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.printf("%.1f ",humidity);
  lcdDblBuf.setTextSize(1);  lcdDblBuf.setCursor(160,67); lcdDblBuf.printf("%%");

  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setCursor(168,29+65);
  lcdDblBuf.setTextColor(ToRGB565(32,32,32));
  lcdDblBuf.printf("%2d",discomfortIndex);
  lcdDblBuf.setCursor(165,25+65);
  if (discomfortIndex < 76)      lcdDblBuf.setTextColor(TFT_CYAN);
  else if (discomfortIndex < 80) lcdDblBuf.setTextColor(TFT_GREEN);
  else if (discomfortIndex < 85) lcdDblBuf.setTextColor(TFT_GREENYELLOW);
  else if (discomfortIndex < 90) lcdDblBuf.setTextColor(TFT_YELLOW);
  else             lcdDblBuf.setTextColor(TFT_LIGHTGREY,TFT_RED);
  lcdDblBuf.printf("%2d",discomfortIndex);
}
/*===================================================================
 * 気圧・標高、温度・湿度、電圧・電流
 */
#define TEXT_RIGHT  177   // 気圧・標高の値の表示左位置
#define UNIT_LEFT   180   // 気圧・標高の単位表示右位置
void DispScrnMode1() {
  lcdDblBuf.fillSprite(ToRGB565(160,64,64));

  //--- 気圧
  sprintf(tmpStr,"%7.1f",pressure);
  i = 2 * M5.Lcd.textWidth(tmpStr,4);           // 書込むlcdDblBufでは値は正しくない、M5.Lcdを使う
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,TEXT_RIGHT - i,0,4);
    lcdDblBuf.setTextSize(1);
    lcdDblBuf.setCursor(UNIT_LEFT,21,4);
    lcdDblBuf.printf("hPa"); 
  //--- 標高
  sprintf(tmpStr,"%7.1f",altitude);
  i = 2 * M5.Lcd.textWidth(tmpStr,4);
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,TEXT_RIGHT - i,40,4);
    lcdDblBuf.setTextSize(1);
    lcdDblBuf.setCursor(UNIT_LEFT,24+36,4);
    lcdDblBuf.printf("m"); 
  //--- 温度・湿度
  sprintf(tmpStr," %5.1fC %4.1f%%",temperature,humidity);
  i = 3 * M5.Lcd.textWidth(tmpStr,1);
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,scrnWidth - i,85,1);
  //--- 電圧・電流
  pwVolt = M5.Axp.GetBatVoltage();
  if (extPW) {
    pwCurt = M5.Axp.GetVBusCurrent(); //　USB電源電流
  } else {
    pwCurt = M5.Axp.GetBatCurrent(); //　バッテリー放電電流
  }
  sprintf(tmpStr,"%4.2fV %3.0fmA",pwVolt,pwCurt);
  i = 3 * M5.Lcd.textWidth(tmpStr,1);
  lcdDblBuf.setTextSize(3);
  lcdDblBuf.setCursor(2, 47+40+23,1);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.drawString(tmpStr,scrnWidth - i,110,1);
}
/*===============================================================================
 * コンパス　日付、標高、気温
 */
#define TEXT_LEFT   240
void DispScrnMode2() {
  bmm150_mag_data value;

  lcdDblBuf.fillSprite(ToRGB565(64,64,96));


  bmm150.read_mag_data();
  value.x = bmm150.raw_mag_data.raw_datax - bmm150Offset.x;
  value.y = bmm150.raw_mag_data.raw_datay - bmm150Offset.y;

  float heading = atan2(value.x, value.y) + PI / 2.0;

  if(heading < 0)    heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
  heading *= 180/PI; 

  lcdDblBuf.setPivot(imgCompWidth/2,imgCompHeight/2);
  copsBuf.pushRotated(&lcdDblBuf,360 - heading);

  lcdDblBuf.setTextSize(2);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.setCursor(145,2, 4);
  lcdDblBuf.printf("%03d",int(heading));

  sprintf(tmpStr,"%02d:%02d",rtcTime.Hours,rtcTime.Minutes);
  i = M5.Lcd.textWidth(tmpStr,4);
  lcdDblBuf.setTextSize(1);
  lcdDblBuf.drawString(tmpStr,scrnWidth - i,50,4);

  sprintf(tmpStr,"%7.1fm",altitude);
  i = M5.Lcd.textWidth(tmpStr,4);           // Font2は高度は幅が入りきらず、１は汚い
  lcdDblBuf.setTextSize(1);
  lcdDblBuf.drawString(tmpStr,scrnWidth - i,70,4);

  sprintf(tmpStr,"%7.1fC",temperature);
  i = M5.Lcd.textWidth(tmpStr,4);
  lcdDblBuf.setTextSize(1);
  lcdDblBuf.drawString(tmpStr,scrnWidth - i,90,4);

/*
  sprintf(tmpStr,"BMP280 %4.1fc",tmp280);
  i = 2 * M5.Lcd.textWidth(tmpStr,1);
  lcdDblBuf.setTextSize(2);
  lcdDblBuf.drawString(tmpStr,scrnWidth - i,110,1);
*/
}
/*============================================================================
 * 気圧履歴
 */
void DispScrnMode3() {
  lcdDblBuf.fillSprite(ToRGB565(128,128,64));

  lcdDblBuf.setTextSize(1);
  lcdDblBuf.setCursor(6,0,2);
  lcdDblBuf.setTextColor(TFT_WHITE);
  lcdDblBuf.printf("  d     h  %4dfhPa  %2dC  %2d%%",(int)pressure,(int)temperature,(int)humidity);
    // 幅広にし見やすく
    lcdDblBuf.setCursor(7,0,2);
    lcdDblBuf.printf("  d     h  %4dfhPa  %2dC  %2d%%",(int)pressure,(int)temperature,(int)humidity);
  lcdDblBuf.setTextSize(2);
  for (i = 0; i < 8; i++) {
    if (i % 2 == 0) lcdDblBuf.setTextColor(TFT_YELLOW);
    else            lcdDblBuf.setTextColor(TFT_WHITE);
    lcdDblBuf.setCursor(3,15+ 15*i,1);
    lcdDblBuf.printf("%2d %2d %5d%3d%3d",presAry[i].day,i*3,presAry[i].pressure,presAry[i].temperature,presAry[i].humidity);
      // 幅広にし見やすく
      lcdDblBuf.setCursor(4,15+ 15*i,1);
      lcdDblBuf.printf("%2d %2d %5d%3d%3d",presAry[i].day,i*3,presAry[i].pressure,presAry[i].temperature,presAry[i].humidity);
    lcdDblBuf.setTextSize(1);
    lcdDblBuf.setCursor(220,15+15*i,2);
    lcdDblBuf.printf("%d",(int)CalcWBGTIndex(presAry[i].temperature,presAry[i].humidity));
      // 幅広にし見やすく
      lcdDblBuf.setCursor(221,15+15*i,2);
      lcdDblBuf.printf("%d",(int)CalcWBGTIndex(presAry[i].temperature,presAry[i].humidity));
    lcdDblBuf.setTextSize(2);
    lcdDblBuf.setCursor(3,15+ 15*i,1);
    switch (presAry[i].pressureUpDn) {
      case PRESSURE_DN:
                        lcdDblBuf.setTextColor(TFT_CYAN);   lcdDblBuf.printf("      v");
                          // 幅広にし見やすく
                          lcdDblBuf.setCursor(4,15+ 15*i,1);
                          lcdDblBuf.setTextColor(TFT_CYAN);   lcdDblBuf.printf("      v");
                        break;
      case PRESSURE_UP:
                        lcdDblBuf.setTextColor(ToRGB565(255,128,128)); lcdDblBuf.printf("      ^");
                          // 幅広にし見やすく
                          lcdDblBuf.setCursor(4,15+ 15*i,1);
                          lcdDblBuf.setTextColor(ToRGB565(255,128,128)); lcdDblBuf.printf("      ^");
                        break;
      default :
                        lcdDblBuf.setTextColor(TFT_ORANGE); lcdDblBuf.printf("       ");
                        break;
    }
  }
}
