
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "char.c"
#include "time.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BS 4096
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned char send[32]="AT+CIPSEND=0,%d\r\n";
const unsigned char res[512]="HTTP/1.1 200 OK\r\nContent-Type: text/html;charset=UTF-8\r\nContent-Length:%d\r\n\r\n%s";
const unsigned char rcontent[800]="<html><head><script>var time=%ld*1000;window.onload=function(){setInterval(function(){var nd=new Date();nd.setTime(time);document.getElementById(\"tm\")\.innerHTML=nd.toLocaleString();time+=1000},1000)}\n\
function st(){\
var a=new Date(document.getElementById(\"dti\").value+\" \"+document.getElementById(\"tmi\").value);\
if(a!=null){\
var ajax=new XMLHttpRequest();\
ajax.open('get','/?t='+Math.floor(a.getTime()/1000));\
ajax.onreadystatechange=function(){window.location.href='/';};\
ajax.send();\
}\
}\
</script><title>CLOCK</title></head><body>Time:<div id=\"tm\"></div><br>SET:<input type=\"date\" id=\"dti\"></input><input type=\"time\" id=\"tmi\"></input><button onclick=\"st()\">SET</button></body></html>        \r\n";
unsigned char close[32]="\r\nAT+CIPCLOSE=0\r\n";
unsigned char ress[1024];
unsigned char con[1024];
uint8_t dm[8][128];
void oled_cmd(uint8_t a){
  HAL_GPIO_WritePin(GPIOA,OLED_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,OLED_DC_Pin, GPIO_PIN_RESET);
  //HAL_Delay(2);
  HAL_SPI_Transmit(&hspi1,&a,1,100);
  HAL_GPIO_WritePin(GPIOA,OLED_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,OLED_DC_Pin, GPIO_PIN_SET);
}
void oled_data(uint8_t a){
  HAL_GPIO_WritePin(GPIOA,OLED_CS_Pin, GPIO_PIN_RESET);
  //HAL_Delay(10);
  HAL_SPI_Transmit(&hspi1,&a,1,100);
  HAL_GPIO_WritePin(GPIOA,OLED_CS_Pin, GPIO_PIN_SET);
}
void oclr(void)
{
    uint8_t page,len;
    for(page=0;page<8;page++)
    {
        oled_cmd(0xB0+page);
        oled_cmd(0X00);
        oled_cmd(0X10);
        for(len=0;len<128;len++)
            oled_data(0x0f);
    }
}
void testo(void)
{
    uint8_t page,len;
    for(page=0;page<8;page++)
    {
        oled_cmd(0xB0+page);
        oled_cmd(0X00);
        oled_cmd(0X10);
        for(len=0;len<128;len++)
            oled_data(page*128+len);
    }
}
void display(void)
{
    uint8_t page,len;
    for(page=0;page<8;page++)
    {
        oled_cmd(0xB0+page);
        oled_cmd(0X00);
        oled_cmd(0X10);
        for(len=0;len<128;len++)
            oled_data(dm[page][len]);
    }
}
time_t timestamp;
int mp=0;//mask position 0-5
void mpd(int a){
  mp=(mp+a)%6;
}
void reflash(){
  int i;
  int timestampp=timestamp+8*3600;
    struct tm *p = localtime(&timestampp);
      char timestr[32];
      sprintf(timestr,"%04d/%02d/%02d      %02d:%02d:%02d",(1900+p->tm_year),1+(p->tm_mon), 1+(p->tm_mday), p->tm_hour, p->tm_min, p->tm_sec);
      for(i=0;timestr[i]!='\0';i++){
        uint8_t mk=0;
        if(mp==0&&i>=0&&i<4) mk=0xff;
        if(mp==1&&i>=5&&i<7) mk=0xff;
        if(mp==2&&i>=8&&i<10) mk=0xff;
        if(mp==3&&i>=16&&i<18) mk=0xff;
        if(mp==4&&i>=19&&i<21) mk=0xff;
        if(mp==5&&i>=22&&i<24) mk=0xff;
        charcpy(i/16,i%16,dm,timestr[i],mk);
      }
      display();
}
void changetime(int a){
  switch(mp){
  case 0:
    timestamp+=365*24*3600*a;
    break;
  case 1:
    timestamp+=30*24*3600*a;
    break;
  case 2:
    timestamp+=24*3600*a;
    break;
  case 3:
    timestamp+=3600*a;
    break;
  case 4:
    timestamp+=60*a;
    break;
  case 5:
    timestamp+=a;
    break;
  }
}
unsigned char ubuf[BS];
int head,tail;
int blen(int head,int tail){
  if(head>=tail)
    return head-tail;
  else
    return BS-(tail-head);
}
int pop(unsigned char *a,int len){
  int i;
  if(blen(head,tail)<len)
    return -1;
  else for(i=0;i<len;i++){
    a[i]=ubuf[(tail+i)%BS];
  }
  tail=(len+tail)%BS;
  return 0;
}
int pop1(unsigned char *a,int len){
  int i;
  if(blen(head,tail)<len)
    return -1;
  else for(i=0;i<len;i++){
    a[i]=ubuf[(tail+i)%BS];
  }
  tail=(1+tail)%BS;
  return 0;
}
int push(unsigned char *a,int len){
  int i;
  if(blen(head,tail)+len>=BS)
    return -1;
  else for(i=0;i<len;i++){
    ubuf[(head+i)%BS]=a[i];
  }
  head=(len+head)%BS;
  return 0;
}
int clrr(){
  tail=head;
  return 0;
}
unsigned char buff;
unsigned char espset[6][40]={"AT+CWMODE=3\r\n","AT+RST\r\n","AT+CWSAP=\"CLOCK\",\"11111111\",1,3\r\n","AT+CIPMUX=1\r\n","AT+CIPSERVER=1,80\r\n","ATE0\r\n"};
unsigned char info[16]="AT+CIFSR\r\n";
unsigned char set2[32]="AT+CIPSERVER=1,80\r\n";
/*void ts2str(unsigned int ts,char[] str){
  int year=ts/(1000*60*60*24*365);
  ts=ts-year/4+(year>30)
}*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //SystemCoreClockUpdate();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  
  
  
  //__HAL_RCC_GPIOA_CLK_EABLE();
  uint8_t spidata[28]={0xAE,0x00,0x40,0xB1,0xFF,0xA1,0xC8,0xA6,0xA8,0x3F,0xD3,0x00,0xD5,0x80,0xD9,0xF1,0xDA,0x12,0xDB,0x40,0x20,0x02,0x8D,0x14,0xA6,0xAF,0xAF};
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  int i,j;
  for(i=0;i<28;i++){
    oled_cmd(spidata[i]);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  timestamp=1539600825;
  //testo();
  /*unsigned char tst[]="0123456789test34  gym487@163.com";
  for(i=0;tst[i]!='\0';i++){
    charcpy(i/16,i%16,dm,tst[i]);
  }*/
  /*for(i=0;i<8;i++){
    for(j=0;j<128;j++){
      dm[i][j]=0x0f;
    }
  }*/
  display();
  
  
  for(i=0;i<5;i++){
    HAL_UART_Transmit(&huart1,(uint8_t*)espset[i],32,100);
    HAL_Delay(200); 
  }
  HAL_UART_Transmit(&huart1,(uint8_t*)info,16,100);
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&buff,1);
  HAL_Delay(200);
  unsigned char ip[20];
  unsigned char aa,bb,cc;
  for(i=0;i<64;i++){
    aa=bb;
    bb=cc;
    pop(&cc,1);
    if(aa=='I'&&bb=='P'&&cc==','){
      //pop(&cc,1);
      for(j=0;j<20;j++){
        pop(&cc,1);
        ip[j]=cc;
      }
    }
  }
  for(i=0;i<16;i++){
    charcpy(3,i,dm,ip[i],0x00);
  }
  display();
  while(pop(&cc,1)==0);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Transmit(&huart1,(uint8_t*)set2,32,100);
  unsigned char cmps[6];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   pop1(cmps,5);
   cmps[5]='\0';
   /* for(i=0;i<5;i++){
          charcpy(2,i,dm,cmps[i],0x00);
    }
    display();*/
    if(strcmp("GET /",cmps)==0){
      HAL_Delay(5);
      unsigned char asdf;
      //unsigned char cmp2[16];
      pop(cmps,4);
      pop(cmps,3);
      cmps[3]='\0';
      for(i=0;i<16;i++){
        charcpy(2,i,dm,cmps[i],0x00);
      }
      display();
      if(strcmp(cmps,"?t=")==0){
        unsigned char asd[15];
        //unsigned char asf[15];
        pop(asd,15);
        //sscanf(asd,"%*[0-9] ",asf);
        sscanf(asd,"%ld ",(long int*)&timestamp);
        
        //display();
        //timestamp=1539600820;
      }
      cmps[0]='\0';
      /*for(i=0;i<16;i++){
        pop(&asdf,1);
        charcpy(2,i,dm,asdf,0x00);
      }
      display();*/
      clrr();
      
      unsigned char sendd[32];
      
      sprintf(con,rcontent,timestamp);
      sprintf(ress,res,strlen(con),con);
      sprintf(sendd,send,strlen(ress));
      HAL_UART_Transmit(&huart1,(uint8_t*)sendd,32,100);
      HAL_Delay(10);
      HAL_UART_Transmit(&huart1,(uint8_t*)ress,strlen(ress),100);
      HAL_Delay(10);
      HAL_UART_Transmit(&huart1,(uint8_t*)close,32,100);
      //HAL_Delay(10);
      HAL_UART_Receive_IT(&huart1,(uint8_t *)&buff,1);
    }
   }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8|OLED_DC_Pin 
                          |OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin KEY3_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin|KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 OLED_DC_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|OLED_DC_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int tms=0;
int k1=1,k2=1,k3=1;
unsigned char dpy[16];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int len;
  if(htim==&htim1){
    tms++;
    int i;
    
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1&&k1==0){
      mpd(1);
      reflash();
    }
    
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)==1&&k3==0){
      changetime(-1);
      reflash();
    }
    
    if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==1&&k2==0){
      changetime(+1);
      reflash();
    }
    
    k1=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
    k2=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
    k3=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
    
    
    if((tms%100)==0)
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,(GPIO_PinState)!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4));
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,(GPIO_PinState)!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4));
    if(tms%200==0){
      timestamp+=1;
      reflash();
     /*if(blen(head,tail)>16){
        pop(dpy,16);
        for(i=0;i<16;i++){
          charcpy(2,i,dm,dpy[i],0x00);
        }
        display();
     }else if(blen(head,tail)>0){
       len=blen(head,tail);
       pop(dpy,len);
        for(i=0;i<len;i++){
          charcpy(2,i,dm,dpy[i],0x00);
        }
        display();
     }*/
    }
    
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  push(&buff,1);
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&buff,1);
  //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,(GPIO_PinState)!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3));
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
