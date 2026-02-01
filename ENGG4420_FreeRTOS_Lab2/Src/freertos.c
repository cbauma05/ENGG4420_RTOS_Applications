/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * For use in ENGG*4420 Real-Time Systems
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/

/* USER CODE BEGIN Variables */
uint8_t RxData[256];
uint32_t data_received;
uint8_t Str4Display[50];

// Task handles
osThreadId TaskGUIHandle;
osThreadId TaskCOMMHandle;
osThreadId TaskBTNHandle;

osMessageQId CommQueueHandle;
osMutexId dataMutexHandle;
osSemaphoreId printSemHandle;

osThreadId Task1Handle;
osThreadId Task2Handle;
osMessageQId myQueue01Handle;

osSemaphoreId myBinarySem01Handle;
osMutexId myMutex01Handle;



/* Function prototypes -------------------------------------------------------*/
void StartTask1(void const * argument);
void StartTask2(void const * argument);

typedef struct
{
	uint8_t Value[10];
	uint8_t Source;
}data;

data DataVCP={"VCP\0",2};
data DataToSend={"Hello\0",1};

/* Function prototypes -------------------------------------------------------*/
int myprintf(const char *format, ...);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* user FunctionPrototypes */
void StartTaskCOMM(void const * argument);
void StartTaskBTN(void const * argument);
void StartTaskGUI(void const * argument);


/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {

  /* add mutexes, queues, semaphores, timers, new tasks/threads,... etc. */

	osMutexDef(dataMutex);
	dataMutexHandle = osMutexCreate(osMutex(dataMutex));

	osThreadDef(TaskCOMM, StartTaskCOMM, osPriorityHigh, 0, 128);
	TaskCOMMHandle = osThreadCreate(osThread(TaskCOMM), NULL);

	osThreadDef(TaskBTN, StartTaskBTN, osPriorityAboveNormal, 0, 128);
	TaskBTNHandle = osThreadCreate(osThread(TaskBTN), NULL);

	osThreadDef(TaskGUI, StartTaskGUI, osPriorityAboveNormal, 0, 128);
	TaskGUIHandle = osThreadCreate(osThread(TaskGUI), NULL);

	/* definition and creation of Task1 */
	osThreadDef(Task1, StartTask1, osPriorityAboveNormal, 0, 128);
	Task1Handle = osThreadCreate(osThread(Task1), NULL);


	/* definition and creation of Task2 */
	osThreadDef(Task2, StartTask2, osPriorityNormal, 0, 128);
	Task2Handle = osThreadCreate(osThread(Task2), NULL);


	// Communication task queue
	osMessageQDef(CommQueue, 1, &DataVCP);
	CommQueueHandle = osMessageCreate(osMessageQ(CommQueue), NULL);

	/* definition and creation of printSem */
	osSemaphoreDef(printSem);
	printSemHandle = osSemaphoreCreate(osSemaphore(printSem), 1);

	/* definition and creation of myQueue01 */
	//--------------------------
	osMessageQDef(myQueue01, 1, &DataToSend);
	myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

	/* definition and creation of myBinarySem01 */
	osSemaphoreDef(myBinarySem01);
	myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

	/* Create the mutex(es) */
	/* definition and creation of myMutex01 */
	osMutexDef(myMutex01);
	myMutex01Handle = osMutexCreate(osMutex(myMutex01));

}

/* Serial Communication Task */
void StartTaskCOMM(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  osEvent vcpValue;
  while(1)
  {
	  vcpValue = osMessageGet(CommQueueHandle, osWaitForever);
	  osMutexWait(dataMutexHandle, 0);
	  memcpy(Str4Display,(char *)(((data *)vcpValue.value.p)->Value), data_received+1);
	  osMutexRelease(dataMutexHandle);
	  myprintf("TaskCOMM received: --------%s\n\r", Str4Display);
	  //myprintf("TaskCOMM running.....\n\r");
  }
}

/* GUI Task */
void StartTaskGUI(void const * argument)
{
  uint8_t buf[sizeof(Str4Display)];
  while(1)
  {
	  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	  BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"   Welcome to ");
	  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	  BSP_LCD_DisplayStringAtLine(2, (uint8_t *)"    ENGG4420");
	  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	  BSP_LCD_DisplayStringAtLine(3, (uint8_t *)"Real Time Design");
	  BSP_LCD_SetTextColor(LCD_COLOR_RED);
	  BSP_LCD_DisplayStringAtLine(5, (uint8_t *)"   (FreeRTOS) ");
//	  printf("...... TaskGUI running\n\r");
	  if(Str4Display[0]!='\0')
	  {
		  osMutexWait(dataMutexHandle, 0);
		  memcpy(buf,Str4Display, data_received+1);
		  Str4Display[0] = '\0';
		  osMutexRelease(dataMutexHandle);
		  BSP_LCD_SetTextColor(LCD_COLOR_RED);
		  BSP_LCD_ClearStringLine(7);
		  BSP_LCD_DisplayStringAtLine(7, buf);
		  printf("GUI task received: %s\n\r", buf);
	  }

	  osDelay(500);
  }
}

/* Button Task */
void StartTaskBTN(void const * argument)
{
  uint8_t count_press = 0;
  while(1)
  {
	  if(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET){
		  myprintf("TaskBTN Button Pressed : %i times \n\r", count_press);
		  count_press++;
		  while(HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN)==GPIO_PIN_SET);
	  }
	  osDelay(10);
  }
}

/* StartTask1 function */
void StartTask1(void const * argument)
{
/* Infinite loop */
for(;;)
{
//	myprintf ("Task1 running .....\n\r");
//	osMessagePut(myQueue01Handle, (uint32_t)&DataToSend, 200);
//	osDelay(2000);
//	myprintf ("Task1 Release Semaphore\n\r");
//	osSemaphoreRelease(myBinarySem01Handle);
//	osDelay(1000);
	osMutexWait(myMutex01Handle, 1000);
	myprintf ("Task1 ..... Mutex Print\n\r");
	osMutexRelease(myMutex01Handle);
	osDelay(2000);
}
}

osEvent retvalue;

/* StartTask2 function */
void StartTask2(void const * argument)
{
/* Infinite loop */
for(;;)
{
//	retvalue = osMessageGet(myQueue01Handle, osWaitForever);
//	myprintf ("Received: %s", (char *)(((data *)retvalue.value.p)->Value));
//	myprintf (" ..... Task2 running\n\r ");
//	osSemaphoreWait(myBinarySem01Handle, osWaitForever);
//	myprintf ("Task2 synchronized\n\r");
	osMutexWait(myMutex01Handle, 1000);
	myprintf ("..... Task2 Mutex Print\n\r");
	osMutexRelease(myMutex01Handle);
	osDelay(2000);
}
}

int myprintf(const char *format, ...)
{
	osSemaphoreWait(printSemHandle, osWaitForever);
	va_list alist;
	va_start(alist, format);
	vprintf(format, alist);
    va_end(alist);
	osSemaphoreRelease(printSemHandle);
	return 0;
}

/******END OF FILE****/
