#include <pt.h>
#include "SmallFS.h"
#include "cbuffer.h"
#include "VGA.h"



struct Pipe{
  unsigned int x;
  unsigned int y;
};

int size_offset= 10;
unsigned int offsetPipe[3][10] = {{0 ,30  , 0  , 20 , 0 , 30, 0 , 9, 6 , 10},
                                  {30, 0 , 15 , 5  , 30, 1 , 20, 9, 15, 0},
                                  {0 , 30 , 12  , 17 , 0 , 10 , 0 , 9, 0 , 20}};

//Audio Variables
unsigned long Index = 0;
unsigned int counter=0;
volatile unsigned char PlayingSound;
volatile unsigned char PlayingActioSound;
unsigned int PipeToCheck=0;
unsigned int width = VGA.getHSize();
unsigned int height = VGA.getVSize();
unsigned int speed = 25;
int w=width-2;
int h=height-23;
int points= -1;
Pipe pipes[3]={{w,w},{w,w},{w,w}}; 
  //Pipe pipes[1]={{w,w}};
Pipe bird={w/2,h/2};
unsigned char fbird[10][11] = {{CYAN,CYAN,CYAN,CYAN,BLACK,BLACK,BLACK,CYAN,CYAN,CYAN,CYAN},
{CYAN,CYAN,CYAN,BLACK,WHITE,WHITE,BLACK,BLACK,CYAN,CYAN,CYAN},
{CYAN,CYAN,BLACK,WHITE,YELLOW,BLACK,WHITE,WHITE,BLACK,CYAN,CYAN},
{CYAN,BLACK,BLACK,YELLOW,YELLOW,BLACK,WHITE,BLACK,BLACK,CYAN,CYAN},
{BLACK,WHITE,WHITE,BLACK,YELLOW,YELLOW,BLACK,BLACK,BLACK,BLACK,CYAN},
{BLACK,YELLOW,WHITE,YELLOW,BLACK,YELLOW,RED,RED,RED,RED,BLACK},
{BLACK,YELLOW,YELLOW,YELLOW,BLACK,BLACK,RED,BLACK,BLACK,BLACK,CYAN},
{CYAN,BLACK,YELLOW,PURPLE,PURPLE,BLACK,RED,RED,RED,BLACK,CYAN},
{CYAN,CYAN,BLACK,PURPLE,PURPLE,PURPLE,BLACK,BLACK,BLACK,CYAN,CYAN},
{CYAN,CYAN,BLACK,BLACK,BLACK,BLACK,BLACK,CYAN,CYAN,CYAN,CYAN}};

unsigned int wide = 15;
unsigned int pipesDistance = 35;
unsigned int lowPipeHeigth = 40;


SmallFSFile audiofile;
SmallFSFile audioAction;

#define FRAME_SIZE  64
struct AudioData {
    unsigned char count;
    unsigned char sample[FRAME_SIZE];
};

CircularBuffer<AudioData, 2> audioBuffer;
AudioData currentData;

//Audio Methods
void DAC_SetOutput(unsigned char data)
{
  unsigned int level0 = ((unsigned int)data) << 8;
  unsigned int level1 = ((unsigned int)data) << 8;
  unsigned int level = (level1 << 16) | level0;

  SIGMADELTADATA = level;
}

void SoundInit(void)
{
  // Enable channel 0 and 1
  SIGMADELTACTL = 0x03;

  // Clear timer counter.
  TMR0CNT = 0;

  // Set up timer , no prescaler.
  TMR0CMP = 0;
  TMR0CTL = (0 << TCTLENA) | (1 << TCTLCCM) | (1 << TCTLDIR) | (1 << TCTLIEN);

  // Enable timer 0 interrupt on mask.
  INTRMASK |= (1 << INTRLINE_TIMER0);

  // Globally enable interrupts.
  INTRCTL = (1 << 0);
}

void SoundOff(void)
{
  DAC_SetOutput(0);

  // Disable timer 0
  TMR0CTL &= ~(_BV(TCTLENA));
}

void AudioFillBuffer()
{
  int r;

  AudioData d;
  while (!audioBuffer.isFull()) {
    r = audiofile.read(&d.sample[0], FRAME_SIZE);
    if (r != 0) {
      d.count = r; // 1 sample per byte
      audioBuffer.push(d);
    } else {
      audiofile.seek(0, SEEK_SET);
    }
  }
}

void AudioActionFillBuffer()
{
  int r;

  AudioData d;
  while (!audioBuffer.isFull()) {
    r = audioAction.read(&d.sample[0], FRAME_SIZE);
    if (r != 0) {
      d.count = r; // 1 sample per byte
      audioBuffer.push(d);
    } else {
      //aqui puedo definir una varible para que continue con el otro audio
      PlayingActioSound=0;
      audioAction.seek(0, SEEK_SET);
      return;
    }
  }
}

void SoundPlay(const char *fileName)
{
  unsigned int sampleCount;
  unsigned int frameRate;

  audiofile = SmallFS.open(fileName);
  audioAction = SmallFS.open("sfx_swooshing.snd");
  audiofile.read(&sampleCount, sizeof(unsigned int));
  audiofile.read(&frameRate, sizeof(unsigned int));

  Serial.print("Sample Count = ");
  Serial.println(sampleCount);

  Serial.print("Frame Rate = ");
  Serial.println(frameRate);

  Index = 0;
  PlayingSound = 1;
  AudioFillBuffer();
  currentData = audioBuffer.pop();

  TMR0CNT = 0;
  TMR0CMP = (CLK_FREQ / frameRate) - 1;

  // Enable timer 0
  TMR0CTL |= 1 << TCTLENA;
}

void _zpu_interrupt ()
{
  if ( TMR0CTL & (1 << TCTLIF)) {
    unsigned char sample, hasData = 1;

    if (Index >= currentData.count) {
      if (audioBuffer.hasData()) {
        Index = 0;
        currentData = audioBuffer.pop();
      } else {
        hasData = 0;
      }
    }

    if (hasData) {
      sample = currentData.sample[Index];
      DAC_SetOutput(sample);
      Index++;
    } else {
      SoundOff();
      PlayingSound = 0;
    }

    /* Clear the interrupt flag on timer register */
    TMR0CTL &= ~(1 << TCTLIF);
    TMR0CNT = 0;
  }
}
//End Audio Methods

static struct pt ptDraw,ptBird; //para cada protothread se debe crear una de estas
volatile unsigned char Start=0;

void setup() {
  VGA.begin(VGAWISHBONESLOT(9),CHARMAPWISHBONESLOT(10));

  pinMode(FPGA_LED_0, OUTPUT); // LED init
  PT_INIT(&ptDraw);  // initialise the two
  PT_INIT(&ptBird);  
  
  //Audio Setup
  Serial.begin(9600);
  SoundInit();

  // Configure sigma-delta output pin.
  // GPIO 0 and GPIO 1 are the outputs
  pinMode(FPGA_J2_6, OUTPUT);
  pinModePPS(FPGA_J2_6, HIGH);
  outputPinForFunction(FPGA_J2_6, 0);

  pinMode(FPGA_J2_7, OUTPUT);
  pinModePPS(FPGA_J2_7, HIGH);
  outputPinForFunction(FPGA_J2_7, 0);

  if (SmallFS.begin() < 0) {
    Serial.println("No SmalLFS found, aborting.");
  }
  
  drawBackground();
  drawStartScreen();
  drawBird();
  SoundPlay("guiletheme.snd");
  //End Audio Setup
}



void toggleLED() {
  boolean ledstate = digitalRead(FPGA_LED_0); // get LED state
  ledstate ^= 1;   // toggle LED state using xor
  digitalWrite(FPGA_LED_0, ledstate); // write inversed state back
}

void drawBackground(){
    VGA.clear();
    VGA.setBackgroundColor(BLUE);

    VGA.setColor( WHITE );
    VGA.drawRect( 0, 1, 1, height-1 ); //izquierda
    VGA.drawRect( width-1, 1, 1, height-1 ); //derecha
    
    VGA.drawRect( 0, 1, width-1, 1 ); //arriba
    VGA.drawRect( 0, height-1, width-1, 1 );//abajo

    VGA.setColor( CYAN );
    VGA.drawRect( 1,2, width-2, height-23 ); //CIELO  
    VGA.setColor( GREEN );
    VGA.drawRect( 1,height-23, width-2, 2 ); //BORDER SUELO    
    VGA.setColor( 44,77,81 );
    VGA.drawRect( 1,height-21, width-2, 20 ); //SUELO
}

void eraseBird(){
  VGA.setColor( CYAN );
    VGA.drawRect(bird.x, bird.y,11, 10 );
}

void drawBird(){
  VGA.writeArea( bird.x, bird.y, 11, 10, *fbird );
}

void drawStartScreen(){
  VGA.setColor(RED);
  VGA.printtext(45,2,"FLAPPY BIRD",true);
  VGA.setColor(WHITE);
  VGA.printtext(10,10,"PRESS B1 TO START",true);
  if(points>-1){
    VGA.setColor(RED);
    VGA.printtext(45,18,"GAME OVER",true);
    char* b="";
    itoa(points,b,10);
    points=0;
    VGA.printtext(50,26,"SCORE:",true);
    VGA.printtext(75,34,b,true);
  }
}
int c0=0,c1=0,c2=0,c3=0;
int offset=offsetPipe[0][c0];


void drawPipes(unsigned int toDraw=0){
    
    if(toDraw == 0){
       c0 = c1;
    }
    if(toDraw == 1){
       c0 = c2;
    }
    if(toDraw == 2){
       c0 = c3;
    }
    
    offset = offsetPipe[toDraw][c0];
      if(pipes[toDraw].x > 0){
        VGA.setColor( GREEN );
        VGA.drawRect( pipes[toDraw].x, 2, 1, h - pipesDistance - lowPipeHeigth + offset ); //Top Pipe  
        VGA.drawRect( pipes[toDraw].x, h - lowPipeHeigth + offset , 1 , lowPipeHeigth - offset); //Bottom Pipe 
        pipes[toDraw].x-=1;
      }
     
     if(pipes[toDraw].x<=w-15){
       VGA.setColor( CYAN );
       VGA.drawRect( pipes[toDraw].y, 2, 1, h-pipesDistance - lowPipeHeigth + offset ); //Top Pipe  
       VGA.drawRect( pipes[toDraw].y, h - lowPipeHeigth + offset, 1 , lowPipeHeigth - offset ); //Bottom Pipe 
       pipes[toDraw].y-=1;
     }    
    
     
     if(pipes[toDraw].x==0 && pipes[toDraw].y==0){
       if(toDraw==0){
         c1+=1;
         if(c1 >= 10){
           c1 = 0;
         }
         //c0=c1;
       }
       if(toDraw == 1){
         c2 += 1;
         if(c2 >= 10){
           c2 = 0;
         }
         //c0=c2;
       }
       if(toDraw == 2){
         c3 += 1;
         if(c3 >= 10){
           c3 = 0;
         }
         //c0=c3;
       }
       pipes[toDraw].x = w;
       pipes[toDraw].y = w;
     }
}

void GameOver(){
  bird.x =w/2;
  bird.y =h/2;
  PipeToCheck=0;
  speed = 25;
  counter=0;
  int i = 2;
    while(i>=0){
      pipes[i].x = w;
      pipes[i].y = w;
      i-=1;
    }
    
        delay(3000);
        drawBackground();
        drawStartScreen();
        drawBird();
        delay(2000);
        SoundInit();
        SoundPlay("guiletheme.snd");
        Start=0;
        points=-1;
        drawBackground();
        drawBird();
}

static int MoveBird(struct pt *pt, int interval){
 static unsigned long timestamp = 0;
  PT_BEGIN(pt);
 
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    volatile unsigned char die=0;
    if(Start){
      eraseBird();
            
      if((digitalRead(FPGA_BTN_0) || digitalRead(FPGA_GPIO_9))){
        //points++;
        bird.y-=2;
      }else{
        bird.y+=1;
      }
      
      //Verificacion en y
       if(VGA.getPixel( bird.x+6,bird.y) == GREEN){
          VGA.setColor( BLUE );
          VGA.drawRect( bird.x+6,bird.y, 1, 1);
        die=1;
      }
      
      if(VGA.getPixel( bird.x+8,bird.y+1) == GREEN){
          VGA.setColor( BLUE );
          VGA.drawRect(  bird.x+8,bird.y+1, 1, 1);
        die=1;
      }
      
      if(VGA.getPixel( bird.x+10,bird.y+5) == GREEN){
          VGA.setColor( BLUE );
          VGA.drawRect( bird.x+10,bird.y+5, 1, 1);
         die=1;
      }
      
      if(VGA.getPixel( bird.x+9,bird.y+3) == GREEN){
          VGA.setColor( BLUE );
          VGA.drawRect( bird.x+9,bird.y+3, 1, 1);
         die=1;
      }
      
      if(VGA.getPixel( bird.x+10,bird.y+7) == GREEN){
          VGA.setColor( BLUE );
          VGA.drawRect( bird.x+10,bird.y+7, 1, 1);
         die=1;
      }
      
      if(VGA.getPixel( bird.x+9,bird.y+8) == GREEN){
         die=1;
      }
      
      if(VGA.getPixel( bird.x+8,bird.y+10) == GREEN){
         die=1;
      }
      
      if(VGA.getPixel( bird.x+6,bird.y+10) == GREEN){
         die=1;
      }
      
      if(VGA.getPixel( bird.x+3,bird.y+8) == GREEN){
         die=1;
      }
      
      if(VGA.getPixel( bird.x+2,bird.y+8) == GREEN){
         die=1;
      }
      
      if(bird.y>=h-8 || bird.y<=0){
        GameOver();
      }
    }
    
    drawBird();
    if(die){
      GameOver();
    }
  }
  PT_END(pt); 
 
  
}

/* This function toggles the LED after 'interval' ms passed */
static int Draw(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
 
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    
    if(Start){
      drawPipes();
      
      if(counter>=60){
        drawPipes(1);
      }
      
      if(counter>=120){
        drawPipes(2);
      }
      
      
       if(pipes[PipeToCheck].x+15<bird.x){
         points++;
         if(PipeToCheck<2){
           PipeToCheck++;
         }else{
           PipeToCheck=0;
         }
         
         if(speed>10){
           speed-=5;
         }
       }
      
      if(points>=0){
        char* b="";
        itoa(points,b,10);
        VGA.setColor( RED );
        VGA.printtext(75,15,b);
      }
      
    }
    
    if(counter<=120)
      counter++; 
  }
  PT_END(pt);
}
/* exactly the same as the ptDraw function */
static int protothread2(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    toggleLED();
  }
  PT_END(pt);
}

void loop() {
  
  if((digitalRead(FPGA_BTN_0) || digitalRead(FPGA_GPIO_9)) && !Start){
    Start = 1;
    points =0;
    int i = 2;
    while(i>=0){
      pipes[i].x = w;
      pipes[i].y = w;
      i-=1;
    }    
  }
  
//  if((digitalRead(FPGA_BTN_0) || digitalRead(FPGA_GPIO_9)) && !PlayingActioSound){
//    PlayingActioSound = 1;
//  }
  
  if (PlayingSound) {
    //if(PlayingActioSound){
      //AudioActionFillBuffer();
    //}else{
      AudioFillBuffer();    
    //}
  }
  
  if(Start){
    Draw(&ptDraw,speed); 
    MoveBird(&ptBird,20);
    
  }
   
}
