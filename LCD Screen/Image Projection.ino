#include <Elegoo_GFX.h>    
#include <Elegoo_TFTLCD.h> 
#include <TouchScreen.h>

#define LCD_CS A3 
#define LCD_CD A2 
#define LCD_WR A1 
#define LCD_RD A0 

#define LCD_RESET A4 

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

/******************* UI details */
#define BUTTON_X 70
#define BUTTON_Y 80
#define BUTTON_W 85
#define BUTTON_H 60
#define BUTTON_SPACING_X 15
#define BUTTON_SPACING_Y 10
#define BUTTON_TEXTSIZE 2

// text box where numbers go
#define TEXT_X 10
#define TEXT_Y 10
#define TEXT_W 220
#define TEXT_H 50
#define TEXT_TSIZE 3
#define TEXT_TCOLOR ILI9341_MAGENTA
// the data (phone #) we store in the textfield
#define TEXT_LEN 12
char textfield[TEXT_LEN+1] = "";
uint8_t textfield_i=0;
String name = "";

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

//Touch For New ILI9341 TP
#define TS_MINX 120
#define TS_MAXX 900

#define TS_MINY 70
#define TS_MAXY 920
#define STATUS_X 10
#define STATUS_Y 65
#define BOXSIZE 40


Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

Elegoo_GFX_Button buttons[15];
char buttonlabels[15][2] = {"H", "A", "S", "S", "C", "E"};
uint16_t buttoncolors[15] = {ILI9341_DARKGREEN, ILI9341_RED, ILI9341_NAVY, 
                             ILI9341_ORANGE, ILI9341_GREENYELLOW, ILI9341_PINK};
                             
void setup(void) {
  Serial.begin(9600);

  uint16_t identifier = tft.readID();

  tft.begin(identifier);
  tft.setRotation(2);
  tft.fillScreen(BLACK);

  tft.fillRect(50, 100, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(150, 100, BOXSIZE, BOXSIZE, YELLOW);

  tft.setCursor(30, 250);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Enter your name: ");
  
  Serial.println("Enter your name.");
  name = input_reader();

}
// Print something in the mini status bar with either flashstring
void status(const __FlashStringHelper *msg) {
  tft.fillRect(STATUS_X, STATUS_Y, 240, 8, ILI9341_BLACK);
  tft.setCursor(STATUS_X, STATUS_Y);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.print(msg);
}
// or charstring
void status(char *msg) {
  tft.fillRect(STATUS_X, STATUS_Y, 240, 8, ILI9341_BLACK);
  tft.setCursor(STATUS_X, STATUS_Y);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.print(msg);
}
#define MINPRESSURE 10
#define MAXPRESSURE 1000

char rx_byte = 0;
String rx_str = "";

String input_reader() {
  
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    
    if (rx_byte != '\n') {
      // a character of the string was received
      name += rx_byte;
    }
    else {
      // end of string
      Serial.print("Welcome ");
      Serial.println(name);
      name = "";                // clear the string for reuse
      Serial.println("");
      Serial.println("Enter your name.");
    }
    return name;
  }
}

void Happy(){
  delay(100);
  tft.fillScreen(BLACK);
  tft.fillRect(85, 250, 70, 10, YELLOW);
  
  tft.fillRect(50, 100, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(150, 100, BOXSIZE, BOXSIZE, YELLOW);

  tft.fillRect(40, 190, 10, 10, YELLOW);
  tft.fillRect(190, 190, 10, 10, YELLOW);
  
  tft.fillRect(40, 200, 10, 10, YELLOW);
  tft.fillRect(190, 200, 10, 10, YELLOW);
  
  tft.fillRect(40, 210, 10, 10, YELLOW);
  tft.fillRect(190, 210, 10, 10, YELLOW);
  tft.fillRect(50, 210, 10, 10, YELLOW);
  tft.fillRect(180, 210, 10, 10, YELLOW);
  
  tft.fillRect(50, 220, 10, 10, YELLOW);
  tft.fillRect(180, 220, 10, 10, YELLOW);
  tft.fillRect(60, 220, 10, 10, YELLOW);
  tft.fillRect(170, 220, 10, 10, YELLOW);
  
  tft.fillRect(60, 230, 10, 10, YELLOW);
  tft.fillRect(170, 230, 10, 10, YELLOW);
  tft.fillRect(70, 230, 10, 10, YELLOW);
  tft.fillRect(160, 230, 10, 10, YELLOW);

  tft.fillRect(70, 240, 10, 10, YELLOW);
  tft.fillRect(160, 240, 10, 10, YELLOW);
  tft.fillRect(80, 240, 10, 10, YELLOW);
  tft.fillRect(150, 240, 10, 10, YELLOW);
}

void Angry(){
  delay(100);
  tft.fillScreen(BLACK);

  tft.fillRect(40, 75, 60, 15, RED);
  tft.fillRect(140, 75, 60, 15, RED);
  tft.fillRect(80, 55, 20, 30, RED);
  tft.fillRect(140, 55, 20, 30, RED);
  tft.fillRect(50, 100, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(150, 100, BOXSIZE, BOXSIZE, RED);
 
  tft.fillRect(60, 220, 10, 10, RED);
  tft.fillRect(170, 220, 10, 10, RED);
  
  tft.fillRect(60, 210, 10, 10, RED);
  tft.fillRect(170, 210, 10, 10, RED);
  tft.fillRect(70, 210, 10, 10, RED);
  tft.fillRect(160, 210, 10, 10, RED);

  tft.fillRect(70, 200, 10, 10, RED);
  tft.fillRect(160, 200, 10, 10, RED);
  tft.fillRect(80, 200, 10, 10, RED);
  tft.fillRect(150, 200, 10, 10, RED);

  tft.fillRect(85, 190, 70, 10, RED);
}

void Sad(){
  delay(100);
  tft.fillScreen(BLACK);

  tft.fillRect(50, 100, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(150, 100, BOXSIZE, BOXSIZE, BLUE);
  
  tft.fillRect(50, 230, 10, 10, BLUE);
  tft.fillRect(180, 230, 10, 10, BLUE);
  
  tft.fillRect(50, 220, 10, 10, BLUE);
  tft.fillRect(180, 220, 10, 10, BLUE);
  tft.fillRect(60, 220, 10, 10, BLUE);
  tft.fillRect(170, 220, 10, 10, BLUE);
  
  tft.fillRect(60, 210, 10, 10, BLUE);
  tft.fillRect(170, 210, 10, 10, BLUE);
  tft.fillRect(70, 210, 10, 10, BLUE);
  tft.fillRect(160, 210, 10, 10, BLUE);

  tft.fillRect(70, 200, 10, 10, BLUE);
  tft.fillRect(160, 200, 10, 10, BLUE);
  tft.fillRect(80, 200, 10, 10, BLUE);
  tft.fillRect(150, 200, 10, 10, BLUE);

  tft.fillRect(85, 190, 70, 10, BLUE);

  tft.fillRect(40, 80, 40, 10, BLUE);
  tft.fillRect(150, 80, 40, 10, BLUE);
}

void Surprised(){
  delay(100);
  tft.fillScreen(BLACK);

  tft.fillCircle(80, 110, 30, BLACK);
  tft.fillCircle(160, 110, 30, BLACK);
  tft.fillCircle(80, 110, 25, WHITE);
  tft.fillCircle(160, 110, 25, WHITE);
  tft.fillCircle(80, 110, 5, BLACK);
  tft.fillCircle(160, 110, 5, BLACK);

  tft.fillRect(80, 200, 80, 50, BLACK);
}

void Cry(){
  delay(100);
  tft.fillScreen(BLACK);
  tft.fillRect(50, 100, 55, 15, BLUE);
  tft.fillRect(140, 100, 55, 15, BLUE);
  tft.fillRect(75, 220, 10, 10, BLUE);
  tft.fillRect(155, 220, 10, 10, BLUE);
  
  tft.fillRect(75, 210, 10, 10, BLUE);
  tft.fillRect(155, 210, 10, 10, BLUE);
  tft.fillRect(85, 210, 10, 10, BLUE);
  tft.fillRect(145, 210, 10, 10, BLUE);

  tft.fillRect(85, 200, 10, 10, BLUE);
  tft.fillRect(145, 200, 10, 10, BLUE);
  tft.fillRect(95, 200, 10, 10, BLUE);
  tft.fillRect(135, 200, 10, 10, BLUE);

  tft.fillRect(100, 190, 40, 10, BLUE);
  delay(500);
  tft.fillRect(50, 115, 10, 20, BLUE);
  tft.fillRect(185, 115, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 115, 10, 20, BLACK);
  tft.fillRect(185, 115, 10, 20, BLACK);
  tft.fillRect(50, 135, 10, 20, BLUE);
  tft.fillRect(185, 135, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 135, 10, 20, BLACK);
  tft.fillRect(185, 135, 10, 20, BLACK);
  tft.fillRect(50, 155, 10, 20, BLUE);
  tft.fillRect(185, 155, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 155, 10, 20, BLACK);
  tft.fillRect(185, 155, 10, 20, BLACK);
  tft.fillRect(50, 175, 10, 20, BLUE);
  tft.fillRect(185, 175, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 175, 10, 20, BLACK);
  tft.fillRect(185, 175, 10, 20, BLACK);
  tft.fillRect(50, 195, 10, 20, BLUE);
  tft.fillRect(185, 195, 10, 20, BLUE);
  tft.fillRect(50, 115, 10, 20, BLUE);
  tft.fillRect(185, 115, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 195, 10, 20, BLACK);
  tft.fillRect(185, 195, 10, 20, BLACK);
  tft.fillRect(50, 115, 10, 20, BLACK);
  tft.fillRect(185, 115, 10, 20, BLACK);
  tft.fillRect(50, 215, 10, 20, BLUE);
  tft.fillRect(185, 215, 10, 20, BLUE);
  tft.fillRect(50, 135, 10, 20, BLUE);
  tft.fillRect(185, 135, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 215, 10, 20, BLACK);
  tft.fillRect(185, 215, 10, 20, BLACK);
  tft.fillRect(50, 135, 10, 20, BLACK);
  tft.fillRect(185, 135, 10, 20, BLACK);
  tft.fillRect(50, 235, 10, 20, BLUE);
  tft.fillRect(185, 235, 10, 20, BLUE);
  tft.fillRect(50, 155, 10, 20, BLUE);
  tft.fillRect(185, 155, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 235, 10, 20, BLACK);
  tft.fillRect(185, 235, 10, 20, BLACK);
  tft.fillRect(50, 155, 10, 20, BLACK);
  tft.fillRect(185, 155, 10, 20, BLACK);
  tft.fillRect(50, 255, 10, 20, BLUE);
  tft.fillRect(185, 255, 10, 20, BLUE);
  tft.fillRect(50, 175, 10, 20, BLUE);
  tft.fillRect(185, 175, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 255, 10, 20, BLACK);
  tft.fillRect(185, 255, 10, 20, BLACK);
  tft.fillRect(50, 175, 10, 20, BLACK);
  tft.fillRect(185, 175, 10, 20, BLACK);
  tft.fillRect(50, 275, 10, 20, BLUE);
  tft.fillRect(185, 275, 10, 20, BLUE);
  tft.fillRect(50, 195, 10, 20, BLUE);
  tft.fillRect(185, 195, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 275, 10, 20, BLACK);
  tft.fillRect(185, 275, 10, 20, BLACK);
  tft.fillRect(50, 195, 10, 20, BLACK);
  tft.fillRect(185, 195, 10, 20, BLACK);
  tft.fillRect(50, 295, 10, 20, BLUE);
  tft.fillRect(185, 295, 10, 20, BLUE);
  tft.fillRect(50, 215, 10, 20, BLUE);
  tft.fillRect(185, 215, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 295, 10, 20, BLACK);
  tft.fillRect(185, 295, 10, 20, BLACK);
  tft.fillRect(50, 215, 10, 20, BLACK);
  tft.fillRect(185, 215, 10, 20, BLACK);
  tft.fillRect(50, 235, 10, 20, BLUE);
  tft.fillRect(185, 235, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 235, 10, 20, BLACK);
  tft.fillRect(185, 235, 10, 20, BLACK);
  tft.fillRect(50, 255, 10, 20, BLUE);
  tft.fillRect(185, 255, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 255, 10, 20, BLACK);
  tft.fillRect(185, 255, 10, 20, BLACK);
  tft.fillRect(50, 275, 10, 20, BLUE);
  tft.fillRect(185, 275, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 275, 10, 20, BLACK);
  tft.fillRect(185, 275, 10, 20, BLACK);
  tft.fillRect(50, 295, 10, 20, BLUE);
  tft.fillRect(185, 295, 10, 20, BLUE);
  delay(500);
  tft.fillRect(50, 295, 10, 20, BLACK);
  tft.fillRect(185, 295, 10, 20, BLACK);
}

void Excited(){
  delay(100);
  tft.fillScreen(YELLOW);
  tft.fillRect(85, 250, 70, 10, BLACK);

  tft.fillCircle(80, 110, 25, BLACK);
  tft.fillCircle(160, 110, 25, BLACK);
  tft.fillCircle(80, 110, 23, WHITE);
  tft.fillCircle(160, 110, 23, WHITE);
  tft.fillCircle(80, 110, 15, BLACK);
  tft.fillCircle(160, 110, 15, BLACK);

  tft.fillRect(40, 190, 160, 10, BLACK);
  
  tft.fillRect(40, 200, 10, 10, BLACK);
  tft.fillRect(190, 200, 10, 10, BLACK);
  tft.fillRect(50, 200, 140, 10, WHITE);
  
  tft.fillRect(40, 210, 10, 10, BLACK);
  tft.fillRect(190, 210, 10, 10, BLACK);
  tft.fillRect(50, 210, 10, 10, BLACK);
  tft.fillRect(180, 210, 10, 10, BLACK);
  tft.fillRect(60, 210, 120, 10, WHITE);
  
  tft.fillRect(50, 220, 10, 10, BLACK);
  tft.fillRect(180, 220, 10, 10, BLACK);
  tft.fillRect(60, 220, 10, 10, BLACK);
  tft.fillRect(170, 220, 10, 10, BLACK);
  tft.fillRect(70, 220, 100, 10, WHITE);
  
  tft.fillRect(60, 230, 10, 10, BLACK);
  tft.fillRect(170, 230, 10, 10, BLACK);
  tft.fillRect(70, 230, 10, 10, BLACK);
  tft.fillRect(160, 230, 10, 10, BLACK);
  tft.fillRect(80, 230, 80, 10, WHITE);

  tft.fillRect(70, 240, 10, 10, BLACK);
  tft.fillRect(160, 240, 10, 10, BLACK);
  tft.fillRect(80, 240, 10, 10, BLACK);
  tft.fillRect(150, 240, 10, 10, BLACK);
  tft.fillRect(90, 240, 60, 10, WHITE);
}

void Eat(){
  delay(100);
  tft.fillRect(0, 5, 40, 40, WHITE);
}

void Label(){
  delay(100);
  tft.drawRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, ILI9341_WHITE);
}

int counter = 0;
bool check;

void loop(void) {
  digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint();
  digitalWrite(13, LOW);

  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  //Serial.println("Counter: ");
  //Serial.println(counter);

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {

      if (counter == 2){
        delay(100);
        counter++;
      }
      if (counter == 3){
        delay(100);
        counter=0;
      }
      if (counter == 1){
          p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
          p.y = (tft.height()-map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
          
          for (uint8_t b=0; b<8; b++) {
              if (buttons[b].contains(p.x, p.y)) {
                  if (b == 0) {
                    delay(100);
                    Happy();
                  }
                  if (b == 1) {
                    delay(100);
                    Angry();
                  }
                  if (b == 2) {
                    delay(100);
                    Sad();
                  }
                  if (b == 3) {
                    delay(100);
                    Surprised();
                  }
                  if (b == 4) {
                    delay(100);
                    Cry();
                  }
                  if (b == 5) {
                    delay(100);
                    Excited();
                  }
                  if (b == 6) {
                    delay(100);
                    Label();
                  }
                  if (b == 7) {
                    delay(100);
                    Eat();
                  }
                  counter++;
              }
              
              delay(100);
            }
        }
        
      if (counter == 0)
      {
        delay(100);
        counter++;
        tft.fillScreen(BLACK);
        for (uint8_t row=0; row<3; row++) 
        {
          for (uint8_t col=0; col<2; col++) 
          {
            buttons[col + row*2].initButton(&tft, BUTTON_X+col*(BUTTON_W+BUTTON_SPACING_X), 
                       BUTTON_Y+row*(BUTTON_H+BUTTON_SPACING_Y),    // x, y, w, h, outline, fill, text
                        BUTTON_W, BUTTON_H, ILI9341_WHITE, buttoncolors[col+row*2], ILI9341_WHITE,
                        buttonlabels[col + row*2], BUTTON_TEXTSIZE); 
            buttons[col + row*2].drawButton();
          }
        }    
        tft.setCursor(30, 275);
        tft.setTextColor(WHITE);
        tft.setTextSize(2);
        tft.print("Welcome.");
    }
  }
}
