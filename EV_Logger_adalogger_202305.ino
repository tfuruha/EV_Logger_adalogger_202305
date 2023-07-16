//board type:Adafruit adalogger M0
// 
//------------------------
//ループタイマ制御
//メイン周期タイマ Main  Cyclic timer
#define MAIN_INTERVAL 1000
unsigned long LastMainTime;
bool IsMainIntervalPast(){
  unsigned long ulNowTime = millis();
  if(ulNowTime > LastMainTime + MAIN_INTERVAL){
    LastMainTime=LastMainTime + MAIN_INTERVAL;
    return true;
  }
  else{
    return false;
  }
}
bool Is1sPast(){
  return IsMainIntervalPast();
}
//計測周期タイマ Observation Cyclic timer
#define OBS_INTERVAL 100
unsigned long LastObsTime;
bool IsObsIntervalPast(){
  unsigned long ulNowTime = millis();
  if(ulNowTime >  LastObsTime + OBS_INTERVAL){
    LastObsTime = LastObsTime + OBS_INTERVAL;
    return true;
  }
  else{
    return false;
  }
}



bool b1SPassed;	//1秒経過フラグ
//------------------------
//Watch Dog timer lib.
#include <wdt_samd21.h>

//--- standard library
#include <stdio.h>
#include <LibPrintf.h>    //to use %f at sprintf 

//------------------------
//SDカードのSPI設定
#define SPI_CS 4   //SPI接続のチップセレクト端子 adalogger では4
#include "SPI.h"
#include "SD.h"
bool bSDAvail;
const char* LOGFILENAME = "Logger.csv";
unsigned long iLineIndex;	//デバック用に伸ばしておく
#define LINE_LENGHTH 200  //記録時の1行最大長

//加速度 ADXL345使用,警報ランプ付～～～～～～～～
#include <Wire.h>
//#include <MPU6050_light.h>
#include "tinyADXL345.h"
float acc[3] ;
float angle[3];
#define Xaxis 0
#define Yaxis 1
#define Zaxis 2
bool bAccExit;
#define AccWarnLED  13    //加速度過多警報LEDピン
#define ACC_INT_PIN  5     //INT1入力ピン

const float fAccWarnTH = 0.4;
//ポーリング制御の管理用
#define AccScanInterval 200 //Accスキャン間隔(ms)
unsigned long tLastAccScan; //処理した時刻 (ms),1秒毎処理でmillis()で取得、処理ごとにAccScanIntervalを増分
unsigned short cntAccScan;  //処理した回数(0~4),1秒毎処理で0

//ADXL345 の初期化
void Accsetup(){
  bAccExit= true;
  //ADXL345 接続確認
  if(initADXL345() == false){
    bAccExit=false;
  }
  //ADXL345あり？ セットアップ
  if(bAccExit){
    SetOffsetRegADXL345();
    //ResetOffsetRegADXL345();
    //FIFO OFF (Bypass設定）
    uint8_t val = 0;
    WriteReg(ADXL345_FIFO_CTL,val);

    //測定レンジ±16g 
    //Full resolutionモード設定
    //INT_INVERT 1
    //val = (uint8_t)Range16g | 1<<3U | 1<<5U;
    val = (uint8_t)Range4g  | 1<<3U | 1<<5U;
    WriteReg(ADXL345_DATA_FORMAT,val);
    
    //データレート 200Hz,通常動作
    val=(uint8_t)DATARATE_200_HZ;
    WriteReg(ADXL345_BW_RATE,val);
    
    //ACT dc mode on Y-Axis
    WriteReg(ADXL345_ACT_INACT_CTL,0x20)   ;   //0010 0000
    WriteReg(ADXL345_THRESH_ACT,    8)     ;   //62.5 mg/LSB 0.5G
    WriteReg(ADXL345_INT_MAP   , 0x00)     ;   //set 0 to asigne INT1
    WriteReg(ADXL345_INT_ENABLE, 0x10)     ;   //Activity enable

    //測定モードON
    val = 1 << 3U;
    WriteReg(ADXL345_POWER_CTL,val);
      
	  cntAccScan=0;
	  tLastAccScan=millis();
  }

  //警報LEDピンの設定
  pinMode(AccWarnLED,OUTPUT);
  pinMode(ACC_INT_PIN,INPUT);
  //attachInterrupt(ACC_INT_PIN,IntActiv,RISING ); //外部割込み(FALLING )で呼び出す

}

//ADXL345 からデータを取得する
void get_Accvalue(){
if(!bAccExit) return; //未接続 or 0ms処理待ちなら戻る
  readAcc(&acc[Xaxis],&acc[Yaxis],&acc[Zaxis]); 
  if(acc[Yaxis]>fAccWarnTH || acc[Yaxis]<-fAccWarnTH){
    digitalWrite(AccWarnLED,HIGH);
  }else{
    digitalWrite(AccWarnLED,LOW);
  }
}
//加速度 ～～～～～～～～ ここまで

//～～～GPS処理～～～～～
//GPS 設定
#include <TinyGPS++.h>
TinyGPSPlus gps;
static const uint32_t GPSBaud = 115200; //115,200がおすすめ。9,600だと占有時間過多
//--- define GPS Value --------------
double dLat;// Latitude in degrees (double)
double dLng;// Longitude in degrees (double)
double dAlt;// Altitude in meters (double)
double dKph;// Speed in kilometers per hour (double)
char sDate[20],sTime[20];  // GHPS取得日時文字列 処理に未使用のため
//日時は文字列で取得するので実数は割愛
uint32_t uSV;// Number of satellites in use (u32)
//GPS 動作フラグ
bool isGPSAvail;//GPSデータ取得済フラグ
bool bGPSModAvail;	//GPSモジュール接続済フラグ
bool bLocAvail,bAltAvail,bSpdAvail,bTimAvail,bDayAvail;//センテンス毎 GPSデータ取得済フラグ
char cntGPSNoRespons;	//GPS無反応カウンタ
char cntGPSNoUpdated;
//GPS処理の初期化
void setupGPS(){  //setup GPS
	Serial1.begin(GPSBaud,SERIAL_8N1); //use hardware serial(serial1)
	uSV=0;  //init Number of satellites
	isGPSAvail=false;
	bLocAvail=false;	//センテンス毎 GPSデータ取得済フラグをクリア
	bAltAvail=false;
	bSpdAvail=false;
	bDayAvail=false;
	bTimAvail=false;
	cntGPSNoRespons=0;
	cntGPSNoUpdated=0;
}
//GPSモジュールデータ取得(tinyGPS++使用)
void getGPSData(){//convert gps buffer to data, and check GPS data is updated.
	//緯度・経度の取得 ----------------------------------------------
	if(gps.location.isUpdated()){bLocAvail=true;} 
	if(gps.location.isValid()){
		dLat = gps.location.lat();  dLng = gps.location.lng();	
	}else{
		dLat = -99.;  dLng = -99.;
	}
	//年月日の取得----------------------------------------------
	if(gps.date.isUpdated()){bDayAvail=true;}
	if(gps.date.isValid()){
		sprintf(sDate,"%4d/%02d/%02d",gps.date.year(),gps.date.month(),gps.date.day());   //10 chars
	}else{
		sprintf(sDate,"NoDate");
	}
	//時刻の取得 ----------------------------------------------
	if(gps.time.isUpdated()){bTimAvail=true;}
	if(gps.time.isValid()){
		sprintf(sTime,"%02d:%02d:%02d",gps.time.hour(),gps.time.minute(),gps.time.second());  //8chars
	}else{
		sprintf(sTime,"NoTime");
	}
	//使用衛星数の取得 ----------------------------------------------
	if(gps.satellites.isValid()){
		uSV=gps.satellites.value();
	}else{
	  uSV=0;
	}
	//高度の取得 ----------------------------------------------
	if(gps.altitude.isUpdated()){bAltAvail=true;}
	if(gps.altitude.isValid()){ 
		dAlt=gps.altitude.meters();
	}
	else{
		dAlt=0.;
	}
	//速度の取得 ----------------------------------------------
	if(gps.speed.isUpdated()){bSpdAvail=true;}
	if(gps.speed.isValid()){
		dKph=gps.speed.kmph();
	}else{
		dKph=0.;
	}
}
//～～～GPS処理～～～ここまで～～

//タイマイベントハンドラ
char cntTMR;	//タイマカウンタ
//タイマ処理ハンドラ

//************************ Spd 関連
//共通の変数
unsigned int uRotSpd;//回転数(min-1)
#define SPD_IN_PIN 6 //FGパルス入力 ：D6  [20230502]
unsigned long ulLastCntUS;	//前回値
unsigned long ulNowCntUS;	//今回値
const unsigned long ENDofMICROS = 4294967295	;//micros()の最大値, (2^32 - 1).
float fRotSpdNow;
float fSpeedNow;
unsigned long ulCycleTime;	//周期
unsigned int iDispCylcleTime;
//for debug
bool bPlsRcvd;
#define MAX_INTERVAL 4000000  // 4,000,000 us →14inchで1km/h相当
#define INIT_INTERVAL 5000000
//--------------------------------
// #define SPD_IN_PIN 
//Spdパルス測定機能の初期化
void initRotSpd(){
  ulCycleTime = INIT_INTERVAL;	//周期の初期値 速度計算を0にするため
  pinMode(SPD_IN_PIN,INPUT);  //入力ピンに設定、プルアップあり
  attachInterrupt(digitalPinToInterrupt(SPD_IN_PIN),IntSpdPls,FALLING ); //外部割込み(FALLING )で呼び出す
  //detachInterrupt(digitalPinToInterrupt(SPD_IN_PIN));
  ulLastCntUS = 0;   //最初のパルス入力 (us)
  bPlsRcvd=false; //for debug
}

//IntSpdPls()：パルス入力処理
//外部割込み(FALLING )で呼び出す
//1ms未満はチャタリングとみなして破棄
void IntSpdPls(){
	unsigned long ulTempCnt = micros();
	if(ulTempCnt - ulLastCntUS > 1000){ //1ms未満はチャタリングとみなして破棄
    ulCycleTime = ulTempCnt - ulLastCntUS; //culculate interval
    ulLastCntUS = ulTempCnt ;       //現在値をバックアップ
    bPlsRcvd=true;
	}
}

//getRotSpd(): 回転数を計算する
//引数：無
//返値:無
//micros()の差分(周期(us))を回転数(min-1)に換算する。
//前回値より少ないときは、micros()の返値が4,294,967,295(2^32-1)を超えたと判定し、計算する。
//回転数は1分(60,000,000 micro sec)を周期で割って算出
//1周期 4sec超(15min-1未満)は0min-1にしておく
//回転センサ入力割込み処理
//micros()を使ってパルス間隔を算出
//速度計算 パルス間隔から回転数、速度を算出
void  getRotSpd(void){
  float fCyleTime =(float) ulCycleTime;// callされた時に周期を使う
	//回転数の計算 1分(60,000,000 micro sec)を周期で割る
//	if(fCyleTime<6000000.){// 12sec以上(5min-1,)は回転無判断
if(micros()-ulLastCntUS < MAX_INTERVAL){ //Timeout 処理
	  fRotSpdNow = 60000000. /fCyleTime; //回転数
    fSpeedNow  =  4129387.4/fCyleTime; //速度  14インチ
	}else{
		fRotSpdNow=0. ;    fSpeedNow =0. ;
  }
}

//************************************************ 
//************************ 電流電圧の計算  ************************ 
float fCurr;	//電流(A)
float fVolt;	//電圧(V)

//getCurr():電流値を取得する
//引数:ADCの取得値(unsigned int)
//返値:電流値(float)
//Allegro ACS723出力を5V→3.3Vに分圧後入力
//Vcc/2で0A,66.7mV/A(100mV/Aを2/3に分圧

//USE_ADC12bit
#define ADC0A 2042.3  //2048
#define ADC1A 61.45 //50mV/3333mV*4096
float getCurr(float fADC){
	float fCurrADC=(fADC-(float)ADC0A)/ADC1A;
  if(fCurrADC>20.0){
    fCurrADC=20.0;
  } 
  else if(fCurrADC<-20.0){
    fCurrADC=-20.0;
  } 
	return fCurrADC;
}
//--------------------------------

//--------------------------------
//getVolt():電圧値を取得する
//引数:ADCの取得値(unsigned int)
//返値:電圧値(float)
//80V→3.3Vに分圧してから入力
//#define ADCFS_VOLT 80.0

//#ifdef USE_ADC12bit //12bitADC
#define ADC1V 51.2// 4096/80

float getVolt(float fADC){
	float fVoltADC=fADC/ADC1V;
  if(fVoltADC<0.0){
    fVoltADC=0.0;
  }
  else if(fVoltADC>75.0){
    fVoltADC=75.0;
  }
	return fVoltADC;
}
//************************ 電流・電圧の取得 ************************ 
#define VoltScanPin A1
#define CurrScanPin A2
#define NUMAVEBUFF 8
#define NUMAVE 4
int NumAdcSmp ; //ADC サンプリング数、電流積算したらクリア
int AdcCurr[NUMAVEBUFF],AdcVolt[NUMAVEBUFF];  //移動平均用バッファ 最新が0
//ADCの設定
void setupADC(){
  analogReadResolution(12); //Use 12bits ADC
  for(int i=0;i<NUMAVE;i++){
    AdcVolt[i] = 0;    AdcCurr[i] = 2047;
  }
  NumAdcSmp = 0;
}
//ADC計測(変換は無し)
void getADCVoltCUrr(){
  //iにi-1を代入
  for(int i=NUMAVE; i>0; i--){
    AdcCurr[i] = AdcCurr[i-1];
    AdcVolt[i] = AdcVolt[i-1];
  }
  //0に変換値を入れる
  AdcVolt[0]=analogRead(VoltScanPin);
  AdcCurr[0]=analogRead(CurrScanPin);
  NumAdcSmp++;
}
//平均化と実値変換
void getADCtoVA(){
  if(NumAdcSmp > NUMAVE-1){
    NumAdcSmp = NUMAVE ;
    int SumAdcVolt = 0; int SumAdcCurr =0; 
    for(int i=0;i<NUMAVE ;i++){
      SumAdcVolt=AdcVolt[i] + SumAdcVolt;
      SumAdcCurr=AdcCurr[i] + SumAdcCurr;
    }
    fVolt = getVolt(( (float)SumAdcVolt /(float)NUMAVE ));
    fCurr = getCurr(( (float)SumAdcCurr /(float)NUMAVE ));
  }
  else{
    fVolt = 0.0;  fCurr = 0.0;
  }

}

//************************ 電流積算  ************************ 
long iIntCur;	//積算電流(0.1As) up to 2147483647 .
long iDspIntCurr;	//積算電流(mAh)
//--------------------------------
//IntegerCurr():電流積算処理(積算、表示値作成)
//引数：電流の現在値
//返値：なし
//1秒毎に0.1A単位で積算(積算周期は変更可能)
//表示の都合で上限値は3599964(0.1As){99.999Ah}とする 
#define INT_CURR_CYCLE 1	//積算周期
#define INT_CURR_UPPER 3599964 //積算上限値(0.1As)
void setupIntegerCurr(){
  iIntCur=0;  
}

void IntegerCurr(float fCurrI){
  if(NumAdcSmp > NUMAVE-1){
    NumAdcSmp = 0;
  	int iCurSubA = int(fCurrI*10.);		//電流値を0.1A単位の整数化
	  iIntCur += INT_CURR_CYCLE*iCurSubA;	//積算
	  if(iIntCur > INT_CURR_UPPER){		//上限処理
		  iIntCur = INT_CURR_UPPER;
	  }
	  iDspIntCurr = int(float(iIntCur)/36.);// [mAh]=[0.1As]*100/3600
  }
	
}
//*****スイッチ入力処理******************************************* 
#define BreakSwPin 9
bool bBreakSW;
void setupSwIn(){
  pinMode(BreakSwPin,INPUT_PULLUP);
  bBreakSW=false;
}
void scanSwIn(){
  if(digitalRead(BreakSwPin)==LOW)   bBreakSW=true;
  else bBreakSW=false;
}
//***************************表示の設定 ******************************
#include <U8g2lib.h>
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2( /*rotation=*/ U8G2_R0/*,reset=A6*/); //Amazon 1.5"
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2( /*rotation=*/ U8G2_R2/*,reset=A6*/); //2.42"
//文字列変数
const char strTitle1[]="Cur";
const char strUnit1[]="  A";
const char strTitle2[]="IntCur";
const char strUnit2[]="mAh";
const char strTitle3[]="Spd";
const char strUnit3[]="km/h";
//const char strUnit3[]="rpm";
const char strTitle4[]="Volt";
const char strUnit4[]="  V";
char strData1[8];
char strData2[8];
char strData3[8];
char strData4[8];
//Displayのセットアップ
void SetupDisplay(){
  u8g2.begin();
  u8g2.setContrast(0x4F);   //the brightness for the display,0x00-0xFF
  u8g2.setBusClock(400000);
  //u8g2.setDrawColor(2);//set pixel value
}
//Displayの表示
void ShowDisplay(void){
  //表示用データの文字列の作成
  sprintf(strData1,"%4.1f",fCurr);
  //dtostrf(fCurr,4,1,strData1);
  sprintf(strData2,"%4d",iDspIntCurr);
  //sprintf(strData3,"%4d",(int)fRotSpdNow);
  sprintf(strData3,"%4.1f",fSpeedNow);
  sprintf(strData4,"%4.1f",fVolt);
  //表示内容の作成
  //枠線表示
  u8g2.clearBuffer();
  u8g2.drawHLine(0,32,128);
  u8g2.drawVLine(62,0,64);
	//u8g2.setFont(u8g2_font_finderskeepers_tr);//Width 8,Height 10
	u8g2.setFont(u8g2_font_helvR08_tr );//Width 11,Height 11
	//u8g2.setFont(u8g2_font_t0_14_mr );
  //項目・単位表示
	u8g2.drawStr(  0,10,strTitle1);//
	u8g2.drawStr( 40,10,strUnit1);//
	u8g2.drawStr( 64,10,strTitle2);//
	u8g2.drawStr(104,10,strUnit2);//
	u8g2.drawStr(  0,42,strTitle3);//
	u8g2.drawStr( 38,42,strUnit3);//
	u8g2.drawStr( 64,42,strTitle4);//
	u8g2.drawStr(104,42,strUnit4);//
	
  //u8g2.setFont(u8g2_font_fur17_tn  );//proportional
  //u8g2.setFont(u8g2_font_helvR18_tn );//proportional
	u8g2.setFont(u8g2_font_inr16_mn  );//monospace
  //u8g2.setFont(u8g2_font_VCR_OSD_mn );
	//u8g2.setFont(u8g2_font_t0_22_mn );//monospace
  //データ表示 右寄せ
  byte lenstr=u8g2.getStrWidth(strData1);
	u8g2.drawStr( 60-lenstr,28,strData1);//u8g2.drawStr( 0,31,strData1);//
  lenstr=u8g2.getStrWidth(strData2);
  u8g2.drawStr(124-lenstr,28,strData2);//u8g2.drawStr(64,31,strData2);//
  lenstr=u8g2.getStrWidth(strData3);
	u8g2.drawStr( 60-lenstr,63,strData3);//u8g2.drawStr( 0,63,strData3);//
  lenstr=u8g2.getStrWidth(strData4);
	u8g2.drawStr(116-lenstr,63,strData4);//u8g2.drawStr(64,63,strData4);//
  //Display icons of SD Card & GPS.
	u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
  if (bSDAvail){  	u8g2.drawStr(120,55,"K"); }//with SD
  else{             u8g2.drawStr(120,55," "); }//w/o SD
  
  if(isGPSAvail){   u8g2.drawStr(120,63,"G"); }//with GPS 
  else{             u8g2.drawStr(120,63," "); }//w/o GPS 
  
  u8g2.sendBuffer();  //Send buffer to Display

}
//******************************
//************************************************ 
//1秒毎に実行する処理
unsigned long tEndAcc,tEndADCtoVA,tEndIntCur,tEndDisp,tEndmkREC,tEndSD;
//
void tmr1S(){
	//iLineIndex=int(millis()/10);//HWタイマカウンタの取り込み
	iLineIndex=millis();//HWタイマカウンタの取り込み for debug
  cntAccScan=0;   //acc取得トリガ操作
  get_Accvalue(); //acc取得0msはここでget
  //tEndAcc=millis();  
  getRotSpd();    //回転数取得
  getADCtoVA();   //Volt Curr実値換算
  //tEndADCtoVA=millis();
  scanSwIn();     //SW入力スキャン
  IntegerCurr(fCurr);   //電流積算の初期化
  //tEndIntCur=millis();
  detachInterrupt(digitalPinToInterrupt(SPD_IN_PIN)); //外部割込み中断
  ShowDisplay();  // LCD表示
  //tEndDisp=millis();	
  mkDataRecord(); //書き出し用データ作成
  //tEndmkREC=millis();
	//SD書き出し
	if(bSDAvail && iLineIndex>10){//SD 有&& 起動後10秒以降
		WriteLogSD();
    //tEndSD=millis();
	}
  attachInterrupt(digitalPinToInterrupt(SPD_IN_PIN),IntSpdPls,FALLING ); //外部割込み再開
	WriteDebugTerm();	//シリアルに出す
  wdt_reset ( );//reset the WDT counter.
  LastObsTime=millis(); //計測タイマのリセット

}


//************************ setup()  ************************ 
void setup() {  // put your setup code here, to run once:
	//setup debug port
	Serial.begin(115200);//
	while (!Serial && millis()<3000);//5secでタイムアウト
  Accsetup(); //加速度センサの初期化
  setupGPS(); //GPS Set
  initRotSpd(); //スピード読込の初期化
  setupIntegerCurr();   //電流積算の初期化
  setupADC();   //ADCの初期化
  setupSwIn();   //SW入力の初期化

  
	SetupDisplay();//setup Display
	
	//SDカード初期化
	iLineIndex=0;
	Serial.println("SD-Card initialize...");
	
	if(SD.begin(SPI_CS)){//CSピンを指定して、SDカード接続を開始
	  //SDカードあり処理
		bSDAvail=true;
		Serial.println("Card is avail.");
		File File1 = SD.open(LOGFILENAME, FILE_WRITE);
		if(File1){
			File1.println("Index,Volt,Curr,RotSpd,AccX,AccY,AccZ,BreakSW,Latitude,Longitude,Alt,GpsSpd,GpsDate,GpsTime");//ヘッダ行
			File1.close();
		}
	}else{
		bSDAvail=false;
		Serial.println("Card not readable!");
		
	}
	// Interval in microsecs
	cntTMR=1;
  while(millis()<3000){ //ACCの初期化待ちで3s
    delay(10);
  }
  b1SPassed=false;
  
  LastMainTime=millis();  //1sタイマの初期値
  LastObsTime=millis();

  wdt_init ( WDT_CONFIG_PER_4K ); //Initialize the WDT with a timeout 4sec
}
//************************ 
unsigned long tStartGPS,tEndGPS;

void loop() {  // put your main code here, to run repeatedly:
  
  if(IsObsIntervalPast()){//計測タイミング
    getADCVoltCUrr();
  }

  if(Is1sPast()){ 
	  b1SPassed=true;	//1秒経過フラグをセット
  	if(++cntGPSNoRespons > 60) cntGPSNoRespons=60 ;	//GPS無反応カウンタをインクリメント 上限60
	  if(++cntGPSNoUpdated > 60) cntGPSNoUpdated=60 ; //GPS無更新カウンタをインクリメント 上限60
  }
	//GPSデータの受信確認
  tStartGPS=millis();
	while (Serial1.available() > 0){//Serial1にデータ有中はGPS読み込み
		//char c=Serial1.read();
		if (gps.encode(Serial1.read()/*c*/)){	//GPSデータの解析
			getGPSData();	//GPSデータの取得
			//Serial.print(millis());	Serial.print(F(","));	//for debug
		}
		isGPSAvail=true;//
		cntGPSNoRespons=0;	//GPS無反応カウンタをクリア
	}
  tEndGPS=millis();
  //loopごとの処理
	if(bLocAvail&&bAltAvail&&bSpdAvail&&bTimAvail&&bDayAvail){//GPSデータ取得後の処理
	  cntAccScan=0; //Acc処理回数をクリア
		tmr1S();	//1秒毎に実行する処理
		bLocAvail=false; 	//センテンス毎 GPSデータ取得済フラグをクリア
		bAltAvail=false;
		bSpdAvail=false;
		bDayAvail=false;
		bTimAvail=false;
		cntGPSNoUpdated=0;	//GPS無更新カウンタをクリア
	}
  else if(b1SPassed && cntGPSNoUpdated> 3){ // GPSの更新が3秒無くて、1S経過したときの処理
  	//GPS 未使用時の処理。タイマ割込みでフラグをセットして、メインループで処理する
  	//GPS接続でも、3秒超 UPDATEしなければ、ここに入る
		tmr1S();	//1秒毎に実行する処理
		b1SPassed=false;
		//cntGPSNoRespons=0;
	}
  else{//それ以外はacc処理だけ実施
    //get_Accvalue();
  }
	
}
//データログ・でバック出力
char DataRecord[LINE_LENGHTH];
void mkDataRecord(){
  	/*sprintf(DataRecord,"%10d,%4f,%4f,%6f,%4f,%4f,%4f,%d,%10f,%10f,%6f,%4.1f,%s,%s",
    iLineIndex,               //行番号 max 9999sec(166分)
    fVolt,fCurr,fRotSpdNow,   //電圧,電流,回転数
    acc[Xaxis],acc[Yaxis],acc[Zaxis],   //加速度
    bBreakSW,                 //ブレーキスイッチ 0:false、1:true
    dLat,dLng,dAlt,dKph,           //緯度 経度 高度 速度
    sDate,sTime);*/             //日時
  	sprintf(DataRecord,"%8d,%6.1f,%6.1f,%6d,%6.2f,%6.2f,%6.2f,%2d,%10f,%10f,%6.2f,%5.1f,%s %s",
    iLineIndex,                       //行番号 max 9999sec(166分)         8+1
    fVolt,fCurr,int(fRotSpdNow),      //電圧(+00.0),電流(+00.0),回転数    6*3+3
    //fVolt,fCurr,int(ulCycleTime/1000),
    acc[Xaxis],acc[Yaxis],acc[Zaxis], //加速度(+0.00) x,y,z               6*3+3
    bBreakSW,                         //ブレーキスイッチ 0:false、1:true   2+1
    dLat,dLng,dAlt,dKph,              //緯度() 経度 高度 速度             10+10+6+5+4
    sDate,sTime);                     //日時(10+8)                       10+8+2 

}
void WriteLogSD(){
	File File1 = SD.open(LOGFILENAME, FILE_WRITE);
	if(File1){
		File1.println(DataRecord);
		File1.close();
	}
	else{
		Serial.println("DataRecord could not be written!");
	}
}

void WriteDebugTerm(){
	if(Serial){
    //char strDebug[100];
    //sprintf(strDebug,"%d,%d,%d,%d,%d,%d,%d,%d,%d,",tStartGPS,tEndGPS,iLineIndex,tEndAcc,tEndADCtoVA,tEndIntCur,tEndDisp,tEndmkREC,tEndSD);
    //Serial.println(strDebug);
    //Serial.print(AdcCurr[0]); Serial.print(","); Serial.println(AdcVolt[0]);
		Serial.println(DataRecord);
	}
	
}

