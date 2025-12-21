//the pin to control ROW 
const int row1 = 2; // the number of the row pin 9 
const int row2 = 3; // the number of the row pin 14 
const int row3 = 4; // the number of the row pin 8 
const int row4 = 5; // the number of the row pin 12 
const int row5 = 6; // the number of the row pin 1 
const int row6 = 7; // the number of the row pin 7 
const int row7 = 8; // the number of the row pin 2 
const int row8 = 9; // the number of the row pin 5 
//the pin to control COl 
const int col1 = 10; // the number of the col pin 13 
const int col2 = 11; // the number of the col pin 3 
const int col3 = 12; // the number of the col pin 4 
const int col4 = 13; // the number of the col pin 10 
const int col5 = 14; // the number of the col pin 6 
const int col6 = 15; // the number of the col pin 11 
const int col7 = 16; // the number of the col pin 15 
const int col8 = 17; // the number of the col pin 16 
unsigned char Text[]={0x00,0x1c,0x22,0x22,0x22,0x22,0x22,0x1c};
void Draw_point(unsigned char x,unsigned char y)
{
   clear_();
   digitalWrite(x+2, HIGH);
   digitalWrite(y+10, LOW);
   delayMicroseconds(100);
}
void show_num(void)
{
  unsigned char i,j,data;
  for(i=0;i<8;i++)
  {
    data=Text[i];
    for(j=0;j<8;j++)
    {
      if(data & 0x01)Draw_point(j,i);
      data>>=1;
    }  
  }
}
void setup(){ 
int i = 0 ; 
for(i=2;i<18;i++) 
 { 
   pinMode(i, OUTPUT); 
  }  
  clear_(); 
}
void loop()
{ 
  show_num();    
} 
void clear_(void)
{
  for(int i=2;i<10;i++)
  digitalWrite(i, LOW);
  for(int i=0;i<8;i++)
  digitalWrite(i+10, HIGH);
}
