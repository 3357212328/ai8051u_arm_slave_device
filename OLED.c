#include "AI8051U.H"
#include "OLED.h"
#include "delay.h"
#include "set_io.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

// --- 引脚定义 (AI8051U) ---
// OLED 软件I2C: SCL=P0.0, SDA=P0.1 (开漏输出 + 内部上拉)
#define OLED_SCL_PIN        P00
#define OLED_SDA_PIN        P01
// --- 引脚定义结束 ---

uint8_t xdata OLED_DisplayBuf[8][128]; // The Frame Buffer (大数组放xdata)

// --- Internal Static Functions for I2C Communication ---
static void OLED_W_SCL(uint8_t BitValue) { OLED_SCL_PIN = BitValue; }
static void OLED_W_SDA(uint8_t BitValue) { OLED_SDA_PIN = BitValue; }
static void OLED_I2C_Start(void) { OLED_W_SDA(1); OLED_W_SCL(1); delay_us(1); OLED_W_SDA(0); delay_us(1); OLED_W_SCL(0); }
static void OLED_I2C_Stop(void) { OLED_W_SDA(0); OLED_W_SCL(1); delay_us(1); OLED_W_SDA(1); }
static void OLED_I2C_SendByte(uint8_t Byte) { uint8_t i; for(i=0;i<8;i++){OLED_W_SDA(!!(Byte & (0x80>>i))); OLED_W_SCL(1); delay_us(1); OLED_W_SCL(0); delay_us(1);} OLED_W_SCL(1); delay_us(1); OLED_W_SCL(0); delay_us(1); }
static void OLED_WriteCommand(uint8_t Command) { OLED_I2C_Start(); OLED_I2C_SendByte(0x78); OLED_I2C_SendByte(0x00); OLED_I2C_SendByte(Command); OLED_I2C_Stop(); }
static void OLED_WriteMultiData(uint8_t xdata *Data, uint8_t Count) { uint8_t i; OLED_I2C_Start(); OLED_I2C_SendByte(0x78); OLED_I2C_SendByte(0x40); for(i=0;i<Count;i++){OLED_I2C_SendByte(Data[i]);} OLED_I2C_Stop(); }

// --- Internal Static Helper Functions ---
static void OLED_GPIO_Init(void)
{
    // 开漏输出 + 打开内部上拉 (OLED模块通常自带外部上拉电阻)
    set_io_mode(od_mode, Pin00, Pin01, Pin_End);
    set_io_mode(en_pur, Pin00, Pin01, Pin_End);

    OLED_W_SCL(1);
    OLED_W_SDA(1);
}

static void OLED_SetCursor(uint8_t Page, uint8_t X) { if(Page>7)Page=7; if(X>127)X=127; OLED_WriteCommand(0xB0|Page); OLED_WriteCommand(0x10|((X&0xF0)>>4)); OLED_WriteCommand(0x00|(X&0x0F)); }
static uint32_t OLED_Pow(uint32_t X, uint32_t Y) { uint32_t R=1; while(Y--)R*=X; return R; }
static uint8_t OLED_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy) { int16_t i, j, c = 0; for(i=0,j=nvert-1;i<nvert;j=i++){if(((verty[i]>testy)!=(verty[j]>testy))&& (testx<(float)(vertx[j]-vertx[i])*(testy-verty[i])/(verty[j]-verty[i])+vertx[i]))c=!c;} return c; }
static uint8_t OLED_IsInAngle(int16_t X, int16_t Y, int16_t StartAngle, int16_t EndAngle) { float PA=atan2((float)Y,(float)X)*180.f/3.14159f; if(StartAngle<=EndAngle){if(PA>=StartAngle&&PA<=EndAngle)return 1;}else{if(PA>=StartAngle||PA<=EndAngle)return 1;} return 0; }

// --- Public Functions ---
void OLED_Init(void) { 
    OLED_GPIO_Init(); 
    delay_ms(100);
    OLED_WriteCommand(0xAE); OLED_WriteCommand(0xD5); OLED_WriteCommand(0x80); OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F); OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00); OLED_WriteCommand(0x40); OLED_WriteCommand(0xA1); OLED_WriteCommand(0xC8); OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12); OLED_WriteCommand(0x81); OLED_WriteCommand(0xCF); OLED_WriteCommand(0xD9); OLED_WriteCommand(0xF1); OLED_WriteCommand(0xDB); OLED_WriteCommand(0x30); OLED_WriteCommand(0xA4); OLED_WriteCommand(0xA6); OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14); OLED_WriteCommand(0xAF); 
    OLED_Clear(); 
    OLED_Update(); 
}

void OLED_Update(void) { 
    uint8_t j; 
    for(j=0;j<8;j++){ 
        OLED_SetCursor(j,0); 
        OLED_WriteMultiData(OLED_DisplayBuf[j],(uint8_t)128);
    } 
}

void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height) {
    int16_t j, Page, Page1, StartCol, EndCol;
    uint8_t ActualWidth;
    if(Width==0||Height==0)return;
    StartCol=(X<0)?0:X;
    EndCol=X+Width-1;
    if(EndCol>127)EndCol=127;
    if(StartCol>EndCol)return;
    Page=Y/8;
    Page1=(Y+Height-1)/8;
    if(Y<0&&(Y%8!=0))Page-=1;
    if((Y+Height-1)<0&&((Y+Height-1)%8!=0))Page1-=1;
    if(Page<0)Page=0;
    if(Page1>7)Page1=7;
    if(Page>Page1)return;
    ActualWidth=(uint8_t)(EndCol-StartCol+1);
    for(j=Page;j<=Page1;j++){
        OLED_SetCursor((uint8_t)j,(uint8_t)StartCol);
        OLED_WriteMultiData(&OLED_DisplayBuf[j][StartCol],ActualWidth);
    }
}

void OLED_Clear(void) { 
    memset(OLED_DisplayBuf, 0x00, sizeof(OLED_DisplayBuf)); 
}

void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height) { 
    int16_t i,j;
    if(Width==0||Height==0)return; 
    for(j=Y;j<Y+Height;j++){
        for(i=X;i<X+Width;i++){
            if(i>=0&&i<=127&&j>=0&&j<=63){
                OLED_DisplayBuf[j/8][i]&=~(0x01<<(j%8));
            }
        }
    }
}

void OLED_Reverse(void) { 
    uint8_t i,j;
    for(j=0;j<8;j++){
        for(i=0;i<128;i++){
            OLED_DisplayBuf[j][i]^=0xFF;
        }
    }
}

void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height) { 
    int16_t i,j;
    if(Width==0||Height==0)return; 
    for(j=Y;j<Y+Height;j++){
        for(i=X;i<X+Width;i++){
            if(i>=0&&i<=127&&j>=0&&j<=63){
                OLED_DisplayBuf[j/8][i]^=(0x01<<(j%8));
            }
        }
    }
}

void OLED_DrawPoint(int16_t X, int16_t Y) { 
    if(X>=0&&X<=127&&Y>=0&&Y<=63){
        OLED_DisplayBuf[Y/8][X]|=(0x01<<(Y%8));
    }
}

uint8_t OLED_GetPoint(int16_t X, int16_t Y) { 
    if(X>=0&&X<=127&&Y>=0&&Y<=63){
        return(OLED_DisplayBuf[Y/8][X]&(0x01<<(Y%8)))?1:0;
    }
    return 0;
}

void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    const uint8_t* font_data_ptr;
    uint8_t width, height;
    uint8_t char_idx = 0; // Default to space if char is not found
    uint8_t i, j;

    if (Char >= ' ' && Char <= '~') {
        char_idx = (uint8_t)(Char - ' ');
    }

    if (FontSize == OLED_8X16) {
        width = 8;
        height = 16;
        font_data_ptr = OLED_F8x16[char_idx];
    } else { // Default to 6x8
        width = 6;
        height = 8;
        font_data_ptr = OLED_F6x8[char_idx];
    }

    for (j = 0; j < height; j++) {
        for (i = 0; i < width; i++) {
            int16_t cur_x = X + (int16_t)i;
            int16_t cur_y = Y + (int16_t)j;

            if (cur_x >= 0 && cur_x < 128 && cur_y >= 0 && cur_y < 64) {
                uint8_t font_byte;

                if (FontSize == OLED_8X16) {
                     uint8_t font_byte_col = font_data_ptr[i + (j / 8) * 8];
                     if (font_byte_col & (1 << (j % 8))) {
                        OLED_DrawPoint(cur_x, cur_y);
                     }
                } else { // 6x8 font
                    font_byte = font_data_ptr[i];
                    if ((font_byte >> j) & 0x01) {
                        OLED_DrawPoint(cur_x, cur_y);
                    }
                }
            }
        }
    }
}

void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize) { 
    uint8_t i,cw=(FontSize==OLED_8X16)?8:6; 
    for(i=0;String[i]!='\0';i++){
        OLED_ShowChar(X+i*cw,Y,String[i],FontSize);
    }
}

void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize) { 
    uint8_t i,cw=(FontSize==OLED_8X16)?8:6;
    if(Length==0)return;
    for(i=0;i<Length;i++){
        OLED_ShowChar(X+i*cw,Y,(char)(Number/OLED_Pow(10,Length-i-1)%10+'0'),FontSize);
    }
}

void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize) { 
    uint32_t N1;
    uint8_t cw=(FontSize==OLED_8X16)?8:6;
    if(Length==0)return;
    if(Number>=0){
        OLED_ShowChar(X,Y,'+',FontSize);
        N1=Number;
    }else{
        OLED_ShowChar(X,Y,'-',FontSize);
        N1=-Number;
    }
    OLED_ShowNum(X+cw,Y,N1,Length,FontSize);
}

void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize) { 
    uint8_t i,SN;
    uint8_t cw=(FontSize==OLED_8X16)?8:6;
    if(Length==0)return;
    for(i=0;i<Length;i++){
        SN=(Number>>((Length-1-i)*4))&0x0F;
        OLED_ShowChar(X+i*cw,Y,(char)((SN<10)?(SN+'0'):(SN-10+'A')),FontSize);
    }
}

void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize) { 
    uint8_t i;
    uint8_t cw=(FontSize==OLED_8X16)?8:6;
    if(Length==0)return;
    for(i=0;i<Length;i++){
        OLED_ShowChar(X+i*cw,Y,(char)((Number>>(Length-1-i))&0x01?'1':'0'),FontSize);
    }
}

void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...) {
    char String[128];
    va_list arg;
    va_start(arg,format);
    vsprintf(String,format,arg);
    va_end(arg);
    OLED_ShowString(X,Y,String,FontSize);
}

void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image) { 
    int16_t i,j;
    uint8_t Yp=Y/8,Yo=Y%8,Hb=(Height+7)/8;
    if(X>127||Y>63||X+Width<=0||Y+Height<=0||Width==0||Height==0)return;
    for(j=0;j<Hb;j++){
        for(i=0;i<Width;i++){
            if(X+i>=0&&X+i<128){
                uint8_t db=Image[j*Width+i];
                if(Yp+j>=0&&Yp+j<8){
                    OLED_DisplayBuf[Yp+j][X+i]|=(db<<Yo);
                }
                if(Yo!=0&&Yp+j+1>=0&&Yp+j+1<8){
                    OLED_DisplayBuf[Yp+j+1][X+i]|=(db>>(8-Yo));
                }
            }
        }
    }
}

void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1) { 
    int16_t x,y,dx,dy,d,iE,iNE,t;
    int16_t x0=X0,y0=Y0,x1=X1,y1=Y1;
    uint8_t yf=0,xyf=0;
    if(y0==y1){
        if(x0>x1){t=x0;x0=x1;x1=t;}
        for(x=x0;x<=x1;x++)OLED_DrawPoint(x,y0);
        return;
    }
    if(x0==x1){
        if(y0>y1){t=y0;y0=y1;y1=t;}
        for(y=y0;y<=y1;y++)OLED_DrawPoint(x0,y);
        return;
    }
    if(x0>x1){t=x0;x0=x1;x1=t;t=y0;y0=y1;y1=t;}
    if(y0>y1){y0=-y0;y1=-y1;yf=1;}
    dx=x1-x0;dy=y1-y0;
    if(abs(dy)>abs(dx)){t=x0;x0=y0;y0=t;t=x1;x1=y1;y1=t;xyf=1;dx=x1-x0;dy=y1-y0;}
    iE=2*abs(dy);iNE=2*(abs(dy)-dx);d=2*abs(dy)-dx;
    x=x0;y=y0;
    while(x<=x1){
        if(yf&&xyf)OLED_DrawPoint(y,-x);
        else if(yf)OLED_DrawPoint(x,-y);
        else if(xyf)OLED_DrawPoint(y,x);
        else OLED_DrawPoint(x,y);
        x++;
        if(d<0){d+=iE;}
        else{if(dy>0)y++;else y--;d+=iNE;}
    }
}

void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled) { 
    int16_t i,j;
    if(Width==0||Height==0)return; 
    if(!IsFilled){
        OLED_DrawLine(X,Y,X+Width-1,Y);
        OLED_DrawLine(X,Y+Height-1,X+Width-1,Y+Height-1);
        OLED_DrawLine(X,Y+1,X,Y+Height-2);
        OLED_DrawLine(X+Width-1,Y+1,X+Width-1,Y+Height-2);
    }else{
        for(i=X;i<X+Width;i++){
            for(j=Y;j<Y+Height;j++){
                OLED_DrawPoint(i,j);
            }
        }
    }
}

void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled) { 
    if(!IsFilled){
        OLED_DrawLine(X0,Y0,X1,Y1);
        OLED_DrawLine(X1,Y1,X2,Y2);
        OLED_DrawLine(X2,Y2,X0,Y0);
    } else{
        int16_t minx=X0,miny=Y0,maxx=X0,maxy=Y0;
        int16_t i,j;
        int16_t vx[3];
        int16_t vy[3];
        vx[0]=X0; vx[1]=X1; vx[2]=X2;
        vy[0]=Y0; vy[1]=Y1; vy[2]=Y2;
        if(X1<minx){minx=X1;}if(X2<minx){minx=X2;}
        if(Y1<miny){miny=Y1;}if(Y2<miny){miny=Y2;}
        if(X1>maxx){maxx=X1;}if(X2>maxx){maxx=X2;}
        if(Y1>maxy){maxy=Y1;}if(Y2>maxy){maxy=Y2;} 
        for(i=minx;i<=maxx;i++){
            for(j=miny;j<=maxy;j++){
                if(OLED_pnpoly(3,vx,vy,i,j)){
                    OLED_DrawPoint(i,j);
                }
            }
        }
    }
}

void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled) { 
    int16_t x=Radius,y=0,err=0;
    int16_t x0=X,y0=Y;
    if(Radius==0)return; 
    while(x>=y){
        if(!IsFilled){
            OLED_DrawPoint(x0+x,y0+y);OLED_DrawPoint(x0+y,y0+x);
            OLED_DrawPoint(x0-y,y0+x);OLED_DrawPoint(x0-x,y0+y);
            OLED_DrawPoint(x0-x,y0-y);OLED_DrawPoint(x0-y,y0-x);
            OLED_DrawPoint(x0+y,y0-x);OLED_DrawPoint(x0+x,y0-y);
        }else{
            OLED_DrawLine(x0-x,y0+y,x0+x,y0+y);
            OLED_DrawLine(x0-x,y0-y,x0+x,y0-y);
            OLED_DrawLine(x0-y,y0+x,x0+y,y0+x);
            OLED_DrawLine(x0-y,y0-x,x0+y,y0-x);
        } 
        if(err<=0){y+=1;err+=2*y+1;}
        if(err>0){x-=1;err-=2*x+1;}
    }
}

void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled) { 
    int16_t x=0,y=B;
    int32_t a2=(int32_t)A*A,b2=(int32_t)B*B;
    int32_t fa2=4*a2,fb2=4*b2;
    int32_t sigma;
    int16_t x0=X,y0=Y;
    if(A==0||B==0)return; 
    sigma=2*b2+a2*(1-2*B);
    while(b2*x<=a2*y){
        if(!IsFilled){
            OLED_DrawPoint(x0+x,y0+y);OLED_DrawPoint(x0-x,y0+y);
            OLED_DrawPoint(x0+x,y0-y);OLED_DrawPoint(x0-x,y0-y);
        }else{
            OLED_DrawLine(x0-x,y0+y,x0+x,y0+y);
            OLED_DrawLine(x0-x,y0-y,x0+x,y0-y);
        }
        if(sigma>=0){sigma+=fa2*(1-y);y--;}
        sigma+=b2*(4*x+6);x++;
    } 
    sigma=2*a2+b2*(1-2*A);
    x=A;y=0;
    while(a2*y<=b2*x){
        if(!IsFilled){
            OLED_DrawPoint(x0+x,y0+y);OLED_DrawPoint(x0-x,y0+y);
            OLED_DrawPoint(x0+x,y0-y);OLED_DrawPoint(x0-x,y0-y);
        }else{
            if(y!=0){
                OLED_DrawLine(x0-x,y0+y,x0+x,y0+y);
                OLED_DrawLine(x0-x,y0-y,x0+x,y0-y);
            }
        }
        if(sigma>=0){sigma+=fb2*(1-x);x--;}
        sigma+=a2*(4*y+6);y++;
    }
}

void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled) {
    int16_t x=Radius,y=0,err=0;
    int16_t x0=X,y0=Y;
    int16_t Px[8];
    int16_t Py[8];
    uint8_t i;
    int16_t di, dj;
    if(Radius==0)return;
    while(x>=y){
        Px[0]=x;  Px[1]=y;  Px[2]=-y; Px[3]=-x; Px[4]=-x; Px[5]=-y; Px[6]=y;  Px[7]=x;
        Py[0]=y;  Py[1]=x;  Py[2]=x;  Py[3]=y;  Py[4]=-y; Py[5]=-x; Py[6]=-x; Py[7]=-y;
        for(i=0;i<8;i++){
            if(OLED_IsInAngle(Px[i],Py[i],StartAngle,EndAngle)){
                if(!IsFilled)OLED_DrawPoint(x0+Px[i],y0+Py[i]);
            }
        }
        if(IsFilled){
            for(di=-x;di<=x;di++){
                for(dj=-y;dj<=y;dj++){
                    if(di*di+dj*dj<=Radius*Radius){
                        if(OLED_IsInAngle(di,dj,StartAngle,EndAngle))OLED_DrawPoint(x0+di,y0+dj);
                    }
                }
            }
        }
        if(err<=0){y+=1;err+=2*y+1;}
        if(err>0){x-=1;err-=2*x+1;}
    }
    if(IsFilled){
        int16_t sx=X+(int16_t)(Radius*cos(StartAngle*3.14159f/180.f)+0.5f);
        int16_t sy=Y+(int16_t)(Radius*sin(StartAngle*3.14159f/180.f)+0.5f);
        int16_t ex=X+(int16_t)(Radius*cos(EndAngle*3.14159f/180.f)+0.5f);
        int16_t ey=Y+(int16_t)(Radius*sin(EndAngle*3.14159f/180.f)+0.5f);
        OLED_DrawLine(X,Y,sx,sy);
        OLED_DrawLine(X,Y,ex,ey);
    }
}