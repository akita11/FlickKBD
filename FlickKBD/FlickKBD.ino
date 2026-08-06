// ソースコードの準備：
// - CH55xduinoをインストール https://github.com/DeqingSun/ch55xduino
// - TouchKey.{c_,h_}を、システムのArduinoフォルダ内のCH55xduinoのlibraries/TouchKey/src に、TouchKey.{c,h}として移動（オリジナルをバックアップの上でコピー）

#define DEV_R // for 1st CH552 (U2), with LED
//#define USBKBD
// when building without USBKBD defined, src/userUsbHidKeyboard must be moved outside the Sketch folder.
#define DEBUG // for touch parameter tuning

#include <TouchKey.h>

#ifdef DEV_R
#include <WS2812.h>
__xdata uint8_t ledData[3];
void setLED(uint8_t r, uint8_t g, uint8_t b){
  set_pixel_for_GRB_LED(ledData, 0, r, g, b);
  neopixel_show_P1_3(ledData, 3);
}
#endif

// 文字モード: ローマ字かな -> アルファベット -> 数字 の循環トグル (C3/R1)
#define MODE_KANA  0
#define MODE_ALPHA 1
#define MODE_NUM   2
uint8_t charMode = MODE_KANA;
// アルファベットモード:大文字(1)/小文字(0)、かなモード:通常(1)/小文字モード(0)
// (どちらもC0/R3で切替、LEDは明(1)/暗(0))
uint8_t altKeyUpper = 1;

#ifdef DEV_R
#define LED_BRIGHT 10
#define LED_DIM     2

// 普段点灯するモード色 (かな=緑/アルファベット=青/数字=赤)、
// かな/アルファベットモードはaltKeyUpperで明暗
void showIdleLED(){
  uint8_t bright = ((charMode == MODE_ALPHA || charMode == MODE_KANA) && !altKeyUpper) ? LED_DIM : LED_BRIGHT;
  if (charMode == MODE_KANA) setLED(0, bright, 0);
  else if (charMode == MODE_ALPHA) setLED(0, 0, bright);
  else setLED(bright, 0, 0);
}

// タッチ検出時: 消灯
void ledTouchOff(){
  setLED(0, 0, 0);
}

// 左右フリック=1回、上下フリック=2回、普段の色でcount回点滅し、
// (タッチ中のため)消灯状態に戻る。離す時にshowIdleLED()で点灯に戻す。
void blinkLED(uint8_t count){
  uint8_t bright = ((charMode == MODE_ALPHA || charMode == MODE_KANA) && !altKeyUpper) ? LED_DIM : LED_BRIGHT;
  for (uint8_t i = 0; i < count; i++){
    if (charMode == MODE_KANA) setLED(0, bright, 0);
    else if (charMode == MODE_ALPHA) setLED(0, 0, bright);
    else setLED(bright, 0, 0);
    delay(80);
    setLED(0, 0, 0u);
    delay(80);
  }
}
#else
void showIdleLED(){}
void ledTouchOff(){}
void blinkLED(uint8_t count){}
#endif

#ifdef USBKBD
  #ifndef USER_USB_RAM
  #error "This example needs to be compiled with a USER USB setting"
  #endif
  #include "src/userUsbHidKeyboard/USBHIDKeyboard.h"
#endif

//       T0 T1 T2 T3 T4 T5 
// DevU: C0 C1 R2 R3 XE YE (USB conn: UP)
// DevR: C2 C3 R0 R1 XE YE (USB conn: RIGHT)

// X------------------ (U4/R4)
//         U0 U1 R0 R1 
// Y       C0 C1 C2 C3 Y(U5/R5)
// | R2 R0       r  r  |
// | R3 R1       r  r  |
// | U2 R2 u  u        |
// | U3 R3 u  u        |
// X------------------

// ============================================================
// TouchKey.c への修正まとめ (2025-07-17)
//
// 【問題1】TIN5/TIN6 が非タッチ時でもタッチ状態のままになる
//   原因A: TouchKey_begin() でポーリングモードにより baseline を初期化するが、
//          ISR 連続スキャンモードでの計測値と TIN5/TIN6 だけ大きくずれる
//          (TIN5: +659差、TIN6: -258差)。TIN6 は起動直後から touchThreshold を
//          超えた状態でスタートし、以後ずっとタッチ判定になる。
//   原因B: baseline の追跡速度 (noiseHalfDelta/noiseCountLimit = 0.2 units/cycle) が
//          TIN5/TIN6 の環境ドリフト速度 (~6 units/cycle) に追いつかない。
//
//   修正1 (TouchKey_begin): IE_TKEY = 1 で ISR を起動した後、2完全スキャンサイクル
//          (12 ISR イベント) 待機し、ISR モードの実計測値で baseline を上書きする。
//          これにより初期ズレが解消され、以後の漸進的ドリフトは Case1 で追跡できる。
//
//   修正2 (TouchKey_Process): touchStuckLimit サイクル (デフォルト500 ≒ 6秒) 以上
//          タッチ状態が続いた場合、baseline を raw に向けて通常より大きく移動させる
//          強制補正機構 (スタックタッチ回復) を追加。修正1 のフォールバックとして機能。
//
// 【問題2】TIN0-3 でタッチを離した後、数秒間タッチ検知が続く
//   原因: タッチ中に Case2 の slow tracking が baseline を少し下方へ移動させる。
//         指を離すと raw はすぐにアンビエントへ戻り raw > baseline (diff > 0) になるが、
//         解除条件が "mdiff > 0 && mdiff < releaseThreshold" であるため、
//         mdiff = 0 の状態では解除条件を満たせない。
//         以後 Case2 upward tracking が baseline をゆっくり引き上げ (~0.2 units/cycle)、
//         数秒後にノイズで mdiff が 1 になって初めて解除される。
//
//   修正3 (TouchKey_Process): タッチ中 (touchKeyPressed セット済み) に diff > 0
//          (raw が baseline を上回った) を検出したとき即座に解除し、baseline を raw に
//          スナップする。CH552 ではタッチ = raw 低下なので、raw が baseline を超えた
//          時点でタッチ終了は確実。タッチ中に raw が baseline を超えるほどの
//          ノイズは実質起きないため誤検出のリスクはない。
// ============================================================

extern volatile __xdata uint16_t touchRawValue[];
extern __xdata uint16_t touchBaseline[];
extern __xdata uint16_t touchMaxHalfDelta;
extern __xdata uint16_t touchThreshold[6];
extern __xdata uint16_t releaseThreshold[6];
extern __xdata uint8_t touchNextCalibrateCycleCounter;
extern __xdata uint8_t touchCycleCounter;

uint8_t touchU = 0;
uint8_t touchR = 0;

uint16_t touch_c = 0;
uint16_t touch_p = 0;

#define COL0 0x001
#define COL1 0x002
#define COL2 0x004
#define COL3 0x008
#define COLY 0x100
#define ROW0 0x010
#define ROW1 0x020
#define ROW2 0x040
#define ROW3 0x080
#define ROWX 0x200
#define COLALL (COL0 | COL1 | COL2 | COL3 | COLY)
#define ROWALL (ROW0 | ROW1 | ROW2 | ROW3 | ROWX)

void setup() {
  // 起動直後、他の初期化より前に明示的にモード色(かな=緑,LED_BRIGHT)を点灯し、
  // 未初期化状態のLEDが誤って明るく見える時間を最小化する
  showIdleLED();
#ifdef DEV_R
  pinMode(13, OUTPUT);
  TouchKey_begin( (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) ); //Enable all 6 channels: TIN0(P1.0), TIN1(P1.1), TIN2(P1.4), TIN3(P1.5), TIN4(P1.6), TIN5(P1.7)
#else
  TouchKey_begin( (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) ); //Enable all 6 channels: TIN0(P1.0), TIN1(P1.1), TIN2(P1.4), TIN3(P1.5), TIN4(P1.6), TIN5(P1.7)
//  TouchKey_begin( (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) ); //Enable all 6 channels: TIN0(P1.0), TIN1(P1.1), TIN2(P1.4), TIN3(P1.5)
#endif
#ifdef DEV_R
/*
  // for v1
  TouchKey_SetMaxHalfDelta(20);    //MHD=5, increase if sensor value are more noisy
  TouchKey_SetNoiseHalfDelta(2);    //NHD=2, If baseline need to adjust at higher rate, increase this value
  TouchKey_SetNoiseCountLimit(10);  //NCL=10, If baseline need to adjust faster, increase this value
//  TouchKey_SetFilterDelayLimit(5);  //FDL=5, make overall adjustment slower
  TouchKey_SetTouchThreshold(1500);  //100, Bigger touch pad can use a bigger value
  TouchKey_SetReleaseThreshold(1000); //80, Smaller than touch threshold
  touchThreshold[4] = 1000; releaseThreshold[4] = 700;
  touchThreshold[5] = 1000; releaseThreshold[5] = 700;
*/
  TouchKey_SetMaxHalfDelta(10);    //MHD=5, increase if sensor value are more noisy
  TouchKey_SetNoiseHalfDelta(2);    //NHD=2, If baseline need to adjust at higher rate, increase this value
  TouchKey_SetNoiseCountLimit(10);  //NCL=10, If baseline need to adjust faster, increase this value
  TouchKey_SetFilterDelayLimit(5);  //FDL=5, make overall adjustment slower
  TouchKey_SetTouchThreshold(250);  //100, Bigger touch pad can use a bigger value
  TouchKey_SetReleaseThreshold(240); //80, Smaller than touch threshold
#else
  // for DEV_U
/*
  // for v1
  TouchKey_SetMaxHalfDelta(20);    //5, increase if sensor value are more noisy
  TouchKey_SetNoiseHalfDelta(2);    //2, If baseline need to adjust at higher rate, increase this value
  TouchKey_SetNoiseCountLimit(10);  //10, If baseline need to adjust faster, increase this value
//  TouchKey_SetFilterDelayLimit(50);  //5, make overall adjustment slower
  TouchKey_SetTouchThreshold(1500);  //100, Bigger touch pad can use a bigger value
  TouchKey_SetReleaseThreshold(1000); //80, Smaller than touch threshold
  touchThreshold[4] = 1000; releaseThreshold[4] = 700;
  touchThreshold[5] = 1000; releaseThreshold[5] = 700;
  */
  TouchKey_SetMaxHalfDelta(10);    //5, increase if sensor value are more noisy
  TouchKey_SetNoiseHalfDelta(2);    //2, If baseline need to adjust at higher rate, increase this value
  TouchKey_SetNoiseCountLimit(10);  //10, If baseline need to adjust faster, increase this value
  TouchKey_SetFilterDelayLimit(5);  //5, make overall adjustment slower
  TouchKey_SetTouchThreshold(250);  //100, Bigger touch pad can use a bigger value
  TouchKey_SetReleaseThreshold(240); //80, Smaller than touch threshold
#endif

  Serial0_begin(9600);
#ifdef USBKBD
  USBInit();
#endif
  showIdleLED();
}

uint8_t st = 0;
uint8_t key1stC = 0, key1stR = 0;
uint8_t keyOffset = 0;

// keyPos (= key1stC + key1stR*4) の特殊キー
#define KP_BS    3  // C3/R0 Backspace
#define KP_MODE  7  // C3/R1 文字モード切替
#define KP_SPC  11  // C3/R2 Space
#define KP_CASE 12  // C0/R3 大文字/小文字切替
#define KP_PUNCT 14 // C2/R3 , . ? !
#define KP_ENT  15  // C3/R3 Enter

// かなモード (ローマ字入力): 子音+母音
char consonant[] = {' ', 'k', 's', '<', 't', 'n', 'h', '*', 'm', 'y', 'r', ' ', 'C', 'w', '?', 'E'};
char vowel[] = {'a', 'i', 'u', 'e', 'o'};

// 数字モードのタッチ(フリック無し)出力。一般的なテンキー配列 (1,2,3/4,5,6/7,8,9/0)
char digitChar[16] = {'1','2','3',0, '4','5','6',0, '7','8','9',0, 0,'0',0,0};

// アルファベットモードのフリック出力 [keyPos][左/上/右/下]
char alphaLetters[16][4] = {
  {0,0,0,0},           // C0R0 (1)
  {'A','B','C',0},     // C1R0 (2)
  {'D','E','F',0},     // C2R0 (3)
  {0,0,0,0},           // C3R0
  {'G','H','I',0},     // C0R1 (4)
  {'J','K','L',0},     // C1R1 (5)
  {'M','N','O',0},     // C2R1 (6)
  {0,0,0,0},           // C3R1
  {'P','Q','R','S'},   // C0R2 (7)
  {'T','U','V',0},     // C1R2 (8)
  {'W','X','Y','Z'},   // C2R2 (9)
  {0,0,0,0},           // C3R2
  {0,0,0,0},           // C0R3
  {0,0,0,0},           // C1R3 (0)
  {0,0,0,0},           // C2R3
  {0,0,0,0},           // C3R3
};

// 数字モードのフリック出力 [keyPos][左/上/右/下]
// ¥ はASCIIで表現できないため、代わりにバックスラッシュ(\)を使用
char numSymbols[16][4] = {
  {'@',0,0,0},           // C0R0 (1)
  {'\\','$',0,0},        // C1R0 (2)
  {'%','\'','#',0},      // C2R0 (3)
  {0,0,0,0},             // C3R0
  {'*',0,0,0},           // C0R1 (4)
  {'+',0,0,0},           // C1R1 (5)
  {'<','=','>',0},       // C2R1 (6)
  {0,0,0,0},             // C3R1
  {'{','}',':',';'},     // C0R2 (7)
  {0,0,0,0},             // C1R2 (8)
  {'^','|','\\',0},      // C2R2 (9)
  {0,0,0,0},             // C3R2
  {0,0,0,0},             // C0R3
  {'~','&',0,0},         // C1R3 (0)
  {0,0,0,0},             // C2R3
  {0,0,0,0},             // C3R3
};

void loop() {
  TouchKey_Process();
  uint8_t touchResult = TouchKey_Get();
#ifdef DEBUG
  for (uint8_t i = 0; i < 6; i++) {
    if (touchResult & (1 << i)) {
      if (i == 0 || i == 1) USBSerial_print(" C ");
      else if (i == 2 || i == 3) USBSerial_print(" R ");
      else if (i == 4) USBSerial_print(" X ");
      else if (i == 5) USBSerial_print(" Y ");
    } else {
      USBSerial_print(" _ ");
    }
  }
  for (uint8_t i = 0; i < 6; i++) {
  	int16_t diff = touchRawValue[i] - touchBaseline[i];
    USBSerial_print(touchRawValue[i]); USBSerial_print("/");
    USBSerial_print(touchBaseline[i]); USBSerial_print(":");
    USBSerial_print(diff); USBSerial_print(" ");
  }
  USBSerial_println("");
  /*
  for (uint8_t i = 4; i < 6; i++) {
    USBSerial_print(touchRawValue[i]); USBSerial_print(","); USBSerial_print(touchBaseline[i]); USBSerial_print(",");
  }
  USBSerial_println("");
  */
#endif
#ifdef DEV_R
  touchR = touchResult;
#else
  touchU = touchResult;
#endif
  // send touchResult another CH552
  Serial0_write(touchResult);

  if (Serial0_available())
  // get touchResult from another CH552
#ifdef DEV_R
  touchU = Serial0_read();
#else
  touchR = Serial0_read();
#endif

// DevU: C0 C1 R2 R3 XE YE (USB conn: UP)
// DevR: C2 C3 R0 R1 XE YE (USB conn: RIGHT)
  touch_c = 0;
  if (touchU & 0x01) touch_c |= COL0;
  if (touchU & 0x02) touch_c |= COL1;
  if (touchR & 0x01) touch_c |= COL2;
  if (touchR & 0x02) touch_c |= COL3;
  if (touchU & 0x20 || touchR & 0x20) touch_c |= COLY;
  if (touchR & 0x04) touch_c |= ROW0;
  if (touchR & 0x08) touch_c |= ROW1;
  if (touchU & 0x04) touch_c |= ROW2;
  if (touchU & 0x08) touch_c |= ROW3;
  if (touchU & 0x10 || touchR & 0x10) touch_c |= ROWX;
#ifndef USBKBD
 #ifdef DEBUG
 #else
/*
  if (touch_c & COL0) USBSerial_print("C0 "); else USBSerial_print("-- ");
  if (touch_c & COL1) USBSerial_print("C1 "); else USBSerial_print("-- ");
  if (touch_c & COL2) USBSerial_print("C2 "); else USBSerial_print("-- ");
  if (touch_c & COL3) USBSerial_print("C3 "); else USBSerial_print("-- ");
  if (touch_c & COLY) USBSerial_print("Y"); else USBSerial_print("-");
  USBSerial_print(" : ");
  if (touch_c & ROW0) USBSerial_print("R0 "); else USBSerial_print("-- ");
  if (touch_c & ROW1) USBSerial_print("R1 "); else USBSerial_print("-- ");
  if (touch_c & ROW2) USBSerial_print("R2 "); else USBSerial_print("-- ");
  if (touch_c & ROW3) USBSerial_print("R3 "); else USBSerial_print("-- ");
  if (touch_c & ROWX) USBSerial_print("X"); else USBSerial_print("-");
  USBSerial_println("");
  */
 #endif
#endif
  uint8_t cntR = 0, cntC = 0;
  if (touch_c & COLY) cntC++;
  if (touch_c & COL0) cntC++;
  if (touch_c & COL1) cntC++;
  if (touch_c & COL2) cntC++;
  if (touch_c & COL3) cntC++;
  if (touch_c & ROWX) cntR++;
  if (touch_c & ROW0) cntR++;
  if (touch_c & ROW1) cntR++;
  if (touch_c & ROW2) cntR++;
  if (touch_c & ROW3) cntR++;

//    C0 C1 C2 C3
// R0 A  KA SA BS
// R1 TA NA HA Typeset
// R2 MA YA RA SPC
// R3 Cp WA ?. ENT

// Typeset: Kana/Alphabet/Number 循環トグル、LEDはモードごとに緑/青/赤
// BS : Backspace ("[BS]")
// Cp : アルファベットモードの Upper/Lower Case 切替 (LED明:大文字/暗:小文字)
// ?. : 押す/左/上/右フリックの順に , . ? !

  if (st == 0 && cntC == 1 && cntR == 1){
    st = 1;
    switch(touch_c & COLALL){
      case COL0 : key1stC = 0; break;
      case COL1 : key1stC = 1; break;
      case COL2 : key1stC = 2; break;
      case COL3 : key1stC = 3; break;
      default: key1stC = 0; break;
    }
    switch(touch_c & ROWALL){
      case ROW0 : key1stR = 0; break;
      case ROW1 : key1stR = 1; break;
      case ROW2 : key1stR = 2; break;
      case ROW3 : key1stR = 3; break;
      default: key1stR = 0; break;
    }
    keyOffset = 0;
    ledTouchOff();
  }

  if (st == 1){
//    USBSerial_print(cntC); USBSerial_print("/"); USBSerial_println(cntR);
    if (cntC == 2){
      // L/R flicked
      if (key1stC == 0 && (touch_c & COLY)) keyOffset = 1;
      else if (key1stC == 0 && (touch_c & COL1)) keyOffset = 3;
      else if (key1stC == 1 && (touch_c & COL0)) keyOffset = 1;
      else if (key1stC == 1 && (touch_c & COL2)) keyOffset = 3;
      else if (key1stC == 2 && (touch_c & COL1)) keyOffset = 1;
      else if (key1stC == 2 && (touch_c & COL3)) keyOffset = 3;
      else if (key1stC == 3 && (touch_c & COL2)) keyOffset = 1;
      st = 2;
      blinkLED(1);
    }
    else if (cntR == 2){
//      USBSerial_print(key1stR); USBSerial_print("--"); USBSerial_println(touch_c, HEX);
      // U/D flicked
      if (key1stR == 0 && (touch_c & ROWX)) keyOffset = 2;
      else if (key1stR == 0 && (touch_c & ROW1)) keyOffset = 4;
      else if (key1stR == 1 && (touch_c & ROW0)) keyOffset = 2;
      else if (key1stR == 1 && (touch_c & ROW2)) keyOffset = 4;
      else if (key1stR == 2 && (touch_c & ROW1)) keyOffset = 2;
      else if (key1stR == 2 && (touch_c & ROW3)) keyOffset = 4;
      else if (key1stR == 3 && (touch_c & ROW2)) keyOffset = 2;
      st = 2;
      blinkLED(2);
    }
  }

  if (st != 0 && cntC == 0 && cntR == 0){
    // released
    uint8_t keyPos = key1stC + key1stR * 4;
    char outPrefix = 0, out1 = 0, out2 = 0;
    uint8_t sendBS = 0, sendSPC = 0, sendENT = 0;

    if (keyPos == KP_BS) {
      sendBS = 1;
    } else if (keyPos == KP_MODE) {
      if (st == 1) {
        charMode++;
        if (charMode > MODE_NUM) charMode = MODE_KANA;
      }
    } else if (keyPos == KP_SPC) {
      sendSPC = 1;
    } else if (keyPos == KP_ENT) {
      sendENT = 1;
    } else if (keyPos == KP_CASE) {
      if (charMode == MODE_NUM) {
        // 数字モード: タッチ/左/上/右/下 = ( ) [ ] 無割当
        switch (keyOffset) {
          case 0: out2 = '('; break;
          case 1: out2 = ')'; break;
          case 2: out2 = '['; break;
          case 3: out2 = ']'; break;
          default: break;
        }
      } else if (st == 1) {
        altKeyUpper = !altKeyUpper;
      }
    } else if (keyPos == KP_PUNCT) {
      if (charMode == MODE_NUM) {
        // 数字モード: タッチ/左/上/右/下 = . , - / 無割当
        switch (keyOffset) {
          case 0: out2 = '.'; break;
          case 1: out2 = ','; break;
          case 2: out2 = '-'; break;
          case 3: out2 = '/'; break;
          default: break;
        }
      } else {
        // 押す/左/上/右フリックの順に , . ? ! (下フリックは未割当)
        switch (keyOffset) {
          case 0: out2 = ','; break;
          case 1: out2 = '.'; break;
          case 2: out2 = '?'; break;
          case 3: out2 = '!'; break;
          default: break;
        }
      }
    } else if (charMode == MODE_KANA) {
      if (keyPos == 9 && keyOffset == 1) {
        // C1R2(YA行) 左フリックのみ ( に差し替え
        out2 = '(';
      } else if (keyPos == 9 && keyOffset == 3) {
        // C1R2(YA行) 右フリックのみ ) に差し替え
        out2 = ')';
      } else if (keyPos == 13) {
        // C1R3(WA行): タッチ=wa, 左=wo, 上=n, 右=- , 下=無割当
        switch (keyOffset) {
          case 0: out1 = 'w'; out2 = 'a'; break;
          case 1: out1 = 'w'; out2 = 'o'; break;
          case 2: out2 = 'n'; break;
          case 3: out2 = '-'; break;
          default: break;
        }
      } else {
        out1 = consonant[keyPos];
        if (out1 == ' ') out1 = 0;
        out2 = vowel[keyOffset];
      }
      // 小文字モード(C0/R3でトグル、altKeyUpper==0)時、
      // 「あいうえお」「つ」「やゆよ」には冒頭に x を前置(ぁぃぅぇぉ/っ/ゃゅょ)
      if (!altKeyUpper) {
        if (keyPos == 0 ||
            (keyPos == 4 && keyOffset == 2) ||
            (keyPos == 9 && keyOffset != 1 && keyOffset != 3)) {
          outPrefix = 'x';
        }
      }
    } else if (charMode == MODE_NUM) {
      if (keyOffset == 0) out2 = digitChar[keyPos];
      else out2 = numSymbols[keyPos][keyOffset - 1];
    } else if (keyPos == 0) {
      // C0R0 (アルファベット): タッチ/左/上/右/下 = @ # & _ -
      switch (keyOffset) {
        case 0: out2 = '@'; break;
        case 1: out2 = '#'; break;
        case 2: out2 = '&'; break;
        case 3: out2 = '_'; break;
        default: break; // 下フリックは無割当
      }
    } else if (keyPos == 13) {
      // C1R3 (アルファベット、1つ左シフト済): タッチ/左/上/右/下 = ' " ( ) -
      switch (keyOffset) {
        case 0: out2 = '\''; break;
        case 1: out2 = '"'; break;
        case 2: out2 = '('; break;
        case 3: out2 = ')'; break;
        default: break; // 下フリックは無割当
      }
    } else { // MODE_ALPHA (その他のキー、1つ左シフト済)
      // 元は左/上/右/下だった alphaLetters[keyPos][0..3] を、
      // そのままタッチ/左/上/右に前詰めし、下フリックは無割当にする
      if (keyOffset < 4) {
        out2 = alphaLetters[keyPos][keyOffset];
        if (out2 != 0 && !altKeyUpper) out2 = out2 - 'A' + 'a';
      }
    }
    //USBSerial_print(st); USBSerial_print("-"); USBSerial_print(key1stC); USBSerial_print("-"); USBSerial_print(key1stR); USBSerial_print("-"); USBSerial_println(keyOffset);
#ifdef USBKBD
    if (sendBS) { Keyboard_write(KEY_BACKSPACE); }
    else if (sendSPC) { Keyboard_write(' '); }
    else if (sendENT) { Keyboard_write(KEY_RETURN); }
    else {
      if (outPrefix != 0) Keyboard_write(outPrefix);
      if (out1 != 0) Keyboard_write(out1);
      if (out2 != 0) Keyboard_write(out2);
    }
#else
    if (sendBS) { USBSerial_println("[BS]"); }
    else if (sendSPC) { USBSerial_println("[SPC]"); }
    else if (sendENT) { USBSerial_println("[ENT]"); }
    else if (outPrefix != 0 || out1 != 0 || out2 != 0) {
      if (outPrefix != 0) USBSerial_print(outPrefix);
      if (out1 != 0) USBSerial_print(out1);
      USBSerial_println(out2);
    }
#endif
    showIdleLED(); // 離したのでモード色の点灯に戻す
    st = 0;
  }
  delay(10);
}
