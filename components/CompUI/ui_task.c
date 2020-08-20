/*##*************************************************************************************************************************************************************
 *      Includes
 **************************************************************************************************************************************************************/
#include "interfaces/ui_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include <driver/gpio.h>
#include "tft.h"
#include "tftspi.h"

#include "CompI2CDrv.h"
#include "CompBattery.h"
#include "CompPWMDrv.h"

static QueueHandle_t hDiagCcontrolQueue = NULL;

uint16_t getNumberOfCovidBeacons( void );
uint16_t getNumberOfActiveCovidBeacons( void );
uint16_t getNumberOfActiveAppleCovidBeacons( void );
uint16_t getNumberOfActiveAndroidCovidBeacons( void );
uint16_t getAgeOfCovidBeacon( uint32_t n );
uint16_t getNumberOfPossibleCovidBeacons( void );
uint16_t getMaxCovidBeacons( void );
uint32_t getTotalSumOfCovidBeacons( void );
uint32_t getMaxAgeOfCovidBeacons( void );
void clrAllCovidBeacons( void );
uint32_t getExposureTimeOfCovidBeacon( uint32_t n );

uint32_t FreeSpaceBaseline = 0UL;
uint32_t BeaconNumber = 0UL;

/*##*************************************************************************************************************************************************************
 *      Intern type declarations
 **************************************************************************************************************************************************************/
static const char *TAG = "diagTask";


/*##*************************************************************************************************************************************************************
 *      Intern function declarations
 **************************************************************************************************************************************************************/

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _doSelfTest( void );

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _init( void );

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _createTask( void );

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _createQueue( void );

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _diag_task();

/*##*************************************************************************************************************************************************************
 *      Function implementation
 **************************************************************************************************************************************************************/

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
extern void UItask_init( void )
{
	_doSelfTest();

	_init();

    _createQueue();     // consider order! first create queue
    _createTask();

    ESP_LOGI(TAG, "DiagTask_init done");
}


/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
extern void UItask_connect( void )
{
    UItask_sendMessage('!'); // trigger initial screen draw
    ESP_LOGI(TAG, "DiagTask_connect done");
}


/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _doSelfTest( void )
{
}


/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _init( void )
{
	// ==========================================================
	// Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST
	#define SPI_BUS TFT_HSPI_HOST
	// ==========================================================

	// ========  PREPARE DISPLAY INITIALIZATION  =========
    esp_err_t ret;

    // ====================================================================
    // === Pins MUST be initialized before SPI interface initialization ===
    // ====================================================================
    TFT_PinsInit();

    // ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

    spi_lobo_device_handle_t spi;

    spi_lobo_bus_config_t buscfg={
        .miso_io_num=M5_PIN_NUM_MISO,				// set SPI MISO pin
        .mosi_io_num=M5_PIN_NUM_MOSI,				// set SPI MOSI pin
        .sclk_io_num=M5_PIN_NUM_CLK,				// set SPI CLK pin
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
		.max_transfer_sz = 6*1024,
    };
    spi_lobo_device_interface_config_t devcfg={
        .clock_speed_hz=8000000,                // Initial clock out at 8 MHz
        .mode=0,                                // SPI mode 0
        .spics_io_num=-1,                       // we will use external CS pin
		.spics_ext_io_num=M5_PIN_NUM_CS,           // external CS pin
		.flags=LB_SPI_DEVICE_HALFDUPLEX,        // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
    };

    vTaskDelay(500 / portTICK_RATE_MS);
	printf("\r\n==============================\r\n");
    printf("TFT display, LoBo 11/2017\r\n");
	printf("==============================\r\n");
    printf("Pins used: miso=%d, mosi=%d, sck=%d, cs=%d\r\n", M5_PIN_NUM_MISO, M5_PIN_NUM_MOSI, M5_PIN_NUM_CLK, M5_PIN_NUM_CS);
	printf("==============================\r\n\r\n");

	// ==================================================================
	// ==== Initialize the SPI bus and attach the LCD to the SPI bus ====

	ret=spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi);
    assert(ret==ESP_OK);
	printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
	disp_spi = spi;

	// ==== Test select/deselect ====
	ret = spi_lobo_device_select(spi, 1);
    assert(ret==ESP_OK);
	ret = spi_lobo_device_deselect(spi);
    assert(ret==ESP_OK);

	printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
	printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");

	// ================================
	// ==== Initialize the Display ====

	printf("SPI: display init...\r\n");
	TFT_display_init();
    printf("OK\r\n");

	// ---- Detect maximum read speed ----
	max_rdclock = find_rd_speed();
	printf("SPI: Max rd speed = %u\r\n", max_rdclock);

    // ==== Set SPI clock used for display operations ====
	spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
	printf("SPI: Changed speed to %u\r\n", spi_lobo_get_speed(spi));

	printf("DisplayController: ");
    switch (tft_disp_type) {
        case DISP_TYPE_ILI9341:
            printf("ILI9341");
            break;
        case DISP_TYPE_ILI9488:
            printf("ILI9488");
            break;
        case DISP_TYPE_ST7789V:
            printf("ST7789V");
            break;
        case DISP_TYPE_ST7735:
            printf("ST7735");
            break;
        case DISP_TYPE_ST7735R:
            printf("ST7735R");
            break;
        case DISP_TYPE_ST7735B:
            printf("ST7735B");
            break;
        default:
            printf("Unknown");
    }
	printf("\r\n");

	font_rotate = 0;
	text_wrap = 0;
	font_transparent = 0;
	font_forceFixed = 0;
	gray_scale = 0;
    TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(LANDSCAPE);
	TFT_setFont(DEJAVU18_FONT, NULL);
	TFT_resetclipwin();

	TFT_fillWindow( TFT_BLACK );

    gpio_pad_select_gpio(M5_PIN_NUM_BTN_A);
    gpio_set_direction(M5_PIN_NUM_BTN_A, GPIO_MODE_INPUT);
	gpio_pad_select_gpio(M5_PIN_NUM_BTN_B);
	gpio_set_direction(M5_PIN_NUM_BTN_B, GPIO_MODE_INPUT);
	gpio_pad_select_gpio(M5_PIN_NUM_BTN_C);
	gpio_set_direction(M5_PIN_NUM_BTN_C, GPIO_MODE_INPUT);

    i2cInit( M5_PIN_NUM_SCL,
             M5_PIN_NUM_SDA,
             100000, // speed
             I2C_PORT_NUM );
}


/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
void vButtonAReleaseTimerCallback(void* arg)
{
    UItask_sendMessage( 'a' );
}


void vButtonBReleaseTimerCallback(void* arg)
{
    UItask_sendMessage( 'b' );
}


/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _createTask( void )
{
    xTaskCreate( _diag_task,
    		     "diag_task",
				 4096, // stack size
				 NULL,
				 (configMAX_PRIORITIES / 2) - 1, // below normal
				 NULL);
}


/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
static void _createQueue( void )
{
    hDiagCcontrolQueue = xQueueCreate ( 50, sizeof ( uint8_t ) );
    if ( NULL == hDiagCcontrolQueue )
    {
//        Error_Handler();
    }
}


void UItask_sendMessage( uint8_t message )
{
    if (pdPASS != xQueueSend( hDiagCcontrolQueue,
                              &message,
                              pdMS_TO_TICKS(15) ) )
    {
        // do someting again overrun
    }
}


/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
typedef void (*updateFieldValueCB_t)(void);
typedef void (*initializeFrameCB_t)(void);

#define MAX_FIELD_NAME_LEN 20
typedef struct {
    uint16_t xOffset; // relative to frame X
    uint16_t fieldLine; // 1 ... n if more than one line of fields, 0 means END OF LIST
    uint16_t maxFieldWidth;
    color_t backgroundColor;
    color_t foregroundColorName;
    color_t foregroundColorValue;
    color_t highlightColorValue;
    char szName[MAX_FIELD_NAME_LEN];
    char szInitialValue[MAX_FIELD_NAME_LEN];
    uint8_t nameFont;
    uint8_t valueFont;
    updateFieldValueCB_t updateFieldValueCB;
    uint16_t yPosOfValue; // calculated and set by drawFrame()
    uint16_t xPosOfValue; // calculated and set by drawFrame()
} framefield_t;

framefield_t *pCurrentFieldToUpdate;

#define MAX_FRAME_TITLE_LEN 20
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    color_t backgroundColor;
    color_t foregroundColor;
    uint16_t xOffsetTitle;
    char szTitle[MAX_FRAME_TITLE_LEN];
    uint8_t titleFont;
    framefield_t *frameFields;
    initializeFrameCB_t initializeFrameCB;
} frameDef_t;

frameDef_t CurrentFrameToDraw;

void drawFrame( frameDef_t frame )
{
    CurrentFrameToDraw = frame;

    TFT_drawRect( frame.x,
                  frame.y,
                  frame.w,
                  frame.h,
                  frame.foregroundColor);

    // print the frame title
    TFT_fillRect( frame.x + frame.xOffsetTitle,
                  frame.y - TFT_getfontheight()/2,
                  TFT_getStringWidth(frame.szTitle),
                  TFT_getfontheight(),
                  frame.backgroundColor );
    TFT_Y = frame.y;
    TFT_setFont( frame.titleFont, NULL );
    _fg = frame.foregroundColor;
    TFT_print( frame.szTitle,
               frame.x + frame.xOffsetTitle,
               frame.y - TFT_getfontheight()/2);
    TFT_print("\n", TFT_X, TFT_Y);
    // now use this as start for the fields
    uint16_t fieldsYStart = TFT_Y;

    // call a function to initialize even more if necessary
    if ( frame.initializeFrameCB != NULL )
    {
        frame.initializeFrameCB();
    }

    // now draw the fields
    framefield_t *pCurrentField = frame.frameFields;
    while ( pCurrentField->fieldLine > 0 )
    {
        TFT_setFont( pCurrentField->nameFont, NULL );
        uint16_t hName = TFT_getfontheight();
        TFT_setFont( pCurrentField->valueFont, NULL );
        uint16_t hValue = TFT_getfontheight();

        pCurrentField->yPosOfValue = fieldsYStart;
        pCurrentField->yPosOfValue += hName;
        pCurrentField->yPosOfValue += (((pCurrentField->fieldLine) - 1) * (hName + hValue)) + 3;

        TFT_setFont( pCurrentField->nameFont, NULL );
        _fg = pCurrentField->foregroundColorName;
        TFT_print( pCurrentField->szName,
                   frame.x + pCurrentField->xOffset,
                   pCurrentField->yPosOfValue - hValue);

        _fg = pCurrentField->foregroundColorValue;
        pCurrentField->xPosOfValue = frame.x + pCurrentField->xOffset;
        TFT_setFont( pCurrentField->valueFont, NULL );
        TFT_print( pCurrentField->szInitialValue,
                   pCurrentField->xPosOfValue,
                   pCurrentField->yPosOfValue);

        // next field
        pCurrentField++;
    }
}

void updateFrame( frameDef_t frame )
{
    CurrentFrameToDraw = frame;
    framefield_t *pCurrentField = frame.frameFields;
    while ( pCurrentField->fieldLine > 0 )
    {
        if ( pCurrentField->updateFieldValueCB != NULL )
        {
            pCurrentFieldToUpdate = pCurrentField;
            (pCurrentField->updateFieldValueCB)();
        }
        // next field
        pCurrentField++;
    }
}


void drawHeader( void )
{
    TFT_setFont( SMALL_FONT, NULL );
    _fg = TFT_WHITE;
    TFT_print("CovidSniffer V1.6", 0, 0 );
}

void drawSoftKeys( void )
{
    TFT_setFont( SMALL_FONT, NULL );
    _fg = TFT_YELLOW;
    TFT_print("DeltaClr", 38, BOTTOM );

    TFT_setFont( SMALL_FONT, NULL );
    _fg = TFT_CYAN;
    TFT_print("CovidClr", 130, BOTTOM );

    TFT_setFont( SMALL_FONT, NULL );
    _fg = TFT_WHITE;
    TFT_print("Backlight", 220, BOTTOM );
}

void updateClock( void )
{
    char szTmp[40];
    TFT_setFont( SMALL_FONT, NULL );
    TickType_t now = (xTaskGetTickCount() * portTICK_PERIOD_MS)  / 1000;
    uint32_t s = now % 60UL;
    now /= 60UL;
    uint32_t m = now % 60UL;
    now /= 60UL;
    uint32_t h = now % 24UL;
    now /= 24UL;
    uint32_t d = now;
    uint8_t batteryLevel = getBatteryLevelPercent( I2C_PORT_NUM );
    char szBattery[7]="....";
    switch (batteryLevel)
    {
        case 100:
            snprintf( szBattery, sizeof(szBattery), "[####}");
            break;
        case 75:
            snprintf( szBattery, sizeof(szBattery), "[###-}");
            break;
        case 50:
            snprintf( szBattery, sizeof(szBattery), "[##--}");
            break;
        case 25:
            snprintf( szBattery, sizeof(szBattery), "[#---}");
            break;
        case 0:
            snprintf( szBattery, sizeof(szBattery), "[----}");
            break;
        default:
            snprintf( szBattery, sizeof(szBattery), "????");
            break;
    }
    snprintf(szTmp, sizeof(szTmp), "%s up %d.%02d:%02d:%02d\n", szBattery, d, h, m, s);
    _fg = TFT_GREEN;
    TFT_print(szTmp, RIGHT, 0);
}

/*!************************************************************************************************************************************************************
 *
 *************************************************************************************************************************************************************/
#define XTITLE    10

#define FIELD3_WIDTH 100
#define FIELD3_GAP ((DEFAULT_TFT_DISPLAY_WIDTH - (3 * FIELD3_WIDTH)) / (2*(3+1)))
#define XFIELD3_1  (( 0 * FIELD3_WIDTH) + ( 1 * FIELD3_GAP ))
#define XFIELD3_2  (( 1 * FIELD3_WIDTH) + ( 3 * FIELD3_GAP ))
#define XFIELD3_3  (( 2 * FIELD3_WIDTH) + ( 5 * FIELD3_GAP ))

#define FIELD4_WIDTH 75
#define FIELD4_GAP ((DEFAULT_TFT_DISPLAY_WIDTH - (4 * FIELD4_WIDTH)) / (2*(4+1)))
#define XFIELD4_1  (( 0 * FIELD4_WIDTH) + ( 1 * FIELD4_GAP ))
#define XFIELD4_2  (( 1 * FIELD4_WIDTH) + ( 3 * FIELD4_GAP ))
#define XFIELD4_3  (( 2 * FIELD4_WIDTH) + ( 5 * FIELD4_GAP ))
#define XFIELD4_4  (( 3 * FIELD4_WIDTH) + ( 7 * FIELD4_GAP ))

#define YFRAME1 20
#define HFRAME1 50

#define YFRAME2 80
#define HFRAME2 144

// die komischen CONST Deklarationen der TFT Lib gehen nicht als Initializer, daher brauchen wir eigene Farben
#define _TFT_BLACK       {   0,   0,   0 }
#define _TFT_BLUE        {   0,   0, 255 }
#define _TFT_GREEN       {   0, 255,   0 }
#define _TFT_CYAN        {   0, 255, 255 }
#define _TFT_RED         { 252,   0,   0 }
#define _TFT_MAGENTA     { 252,   0, 255 }
#define _TFT_YELLOW      { 252, 252,   0 }
#define _TFT_WHITE       { 252, 252, 252 }

void updateFreeSizeFieldCB( void )
{
    char szTmp[20];

    snprintf(szTmp, sizeof(szTmp), "%d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
    _fg = pCurrentFieldToUpdate->foregroundColorValue;
    _bg = pCurrentFieldToUpdate->backgroundColor;
    TFT_setFont( pCurrentFieldToUpdate->valueFont, NULL );
    TFT_fillRect( pCurrentFieldToUpdate->xPosOfValue,
                  pCurrentFieldToUpdate->yPosOfValue,
                  TFT_getStringWidth("0000000"),
                  TFT_getfontheight(),
                  _bg );
    TFT_print( szTmp,
               pCurrentFieldToUpdate->xPosOfValue,
               pCurrentFieldToUpdate->yPosOfValue);
}

void updateLargestFreeSizeFieldCB( void )
{
    char szTmp[20];

    snprintf(szTmp, sizeof(szTmp), "%d", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    _fg = pCurrentFieldToUpdate->foregroundColorValue;
    _bg = pCurrentFieldToUpdate->backgroundColor;
    TFT_setFont( pCurrentFieldToUpdate->valueFont, NULL );
    TFT_fillRect( pCurrentFieldToUpdate->xPosOfValue,
            pCurrentFieldToUpdate->yPosOfValue,
                  TFT_getStringWidth("0000000"),
                  TFT_getfontheight(),
                  _bg );
    TFT_print( szTmp,
               pCurrentFieldToUpdate->xPosOfValue,
               pCurrentFieldToUpdate->yPosOfValue);
}


void updateFreeSizeDeltaFieldCB( void )
{
    char szTmp[20];

    snprintf(szTmp, sizeof(szTmp), "%d",FreeSpaceBaseline - heap_caps_get_free_size(MALLOC_CAP_8BIT));
    _fg = pCurrentFieldToUpdate->foregroundColorValue;
    _bg = pCurrentFieldToUpdate->backgroundColor;
    TFT_setFont( pCurrentFieldToUpdate->valueFont, NULL );
    TFT_fillRect( pCurrentFieldToUpdate->xPosOfValue,
                  pCurrentFieldToUpdate->yPosOfValue,
                  TFT_getStringWidth("0000000"),
                  TFT_getfontheight(),
                  _bg );
    TFT_print( szTmp,
               pCurrentFieldToUpdate->xPosOfValue,
               pCurrentFieldToUpdate->yPosOfValue);
}


void _updateActiveBeaconNrFieldCB( void )
{
    char szTmp[40];

    snprintf(szTmp, sizeof(szTmp), "%d(%d/%d)", getNumberOfActiveCovidBeacons(), getNumberOfActiveAppleCovidBeacons(), getNumberOfActiveAndroidCovidBeacons());
    _fg = pCurrentFieldToUpdate->foregroundColorValue;
    _bg = pCurrentFieldToUpdate->backgroundColor;
    TFT_setFont( pCurrentFieldToUpdate->valueFont, NULL );
    TFT_fillRect( pCurrentFieldToUpdate->xPosOfValue,
                  pCurrentFieldToUpdate->yPosOfValue,
                  TFT_getStringWidth("00000000"),
                  TFT_getfontheight(),
                  _bg );
    TFT_print( szTmp,
    		   pCurrentFieldToUpdate->xPosOfValue,
               pCurrentFieldToUpdate->yPosOfValue);
}


void _updateTotalBeaconNrFieldCB( void )
{
    char szTmp[40];

    snprintf(szTmp, sizeof(szTmp), "%d", getNumberOfCovidBeacons());
    _fg = pCurrentFieldToUpdate->foregroundColorValue;
    _bg = pCurrentFieldToUpdate->backgroundColor;
    TFT_setFont( pCurrentFieldToUpdate->valueFont, NULL );
    TFT_fillRect( pCurrentFieldToUpdate->xPosOfValue,
                  pCurrentFieldToUpdate->yPosOfValue,
                  TFT_getStringWidth("00000000"),
                  TFT_getfontheight(),
                  _bg );
    TFT_print( szTmp,
    		   pCurrentFieldToUpdate->xPosOfValue,
               pCurrentFieldToUpdate->yPosOfValue);
}


void _updateMaxCovidBeaconsCB( void )
{
    char szTmp[40];

    snprintf(szTmp, sizeof(szTmp), "%d", getMaxCovidBeacons());
    _fg = pCurrentFieldToUpdate->foregroundColorValue;
    _bg = pCurrentFieldToUpdate->backgroundColor;
    TFT_setFont( pCurrentFieldToUpdate->valueFont, NULL );
    TFT_fillRect( pCurrentFieldToUpdate->xPosOfValue,
                  pCurrentFieldToUpdate->yPosOfValue,
                  TFT_getStringWidth("00000000"),
                  TFT_getfontheight(),
                  _bg );
    TFT_print( szTmp,
    		   pCurrentFieldToUpdate->xPosOfValue,
               pCurrentFieldToUpdate->yPosOfValue);
}


void _updateTotalSumOfBeaconsCB( void )
{
    char szTmp[40];

    snprintf(szTmp, sizeof(szTmp), "%d", getTotalSumOfCovidBeacons());
    _fg = pCurrentFieldToUpdate->foregroundColorValue;
    _bg = pCurrentFieldToUpdate->backgroundColor;
    TFT_setFont( pCurrentFieldToUpdate->valueFont, NULL );
    TFT_fillRect( pCurrentFieldToUpdate->xPosOfValue,
                  pCurrentFieldToUpdate->yPosOfValue,
                  TFT_getStringWidth("00000000"),
                  TFT_getfontheight(),
                  _bg );
    TFT_print( szTmp,
    		   pCurrentFieldToUpdate->xPosOfValue,
               pCurrentFieldToUpdate->yPosOfValue);
}


///////////////////////////////////////////////////////////////////////////////
void initializeCovidBeaconHistoryCB( void )
{
	int yActive = (CurrentFrameToDraw.y + CurrentFrameToDraw.h) - 1 - ( getMaxAgeOfCovidBeacons() / 2 );
	int yMax = (CurrentFrameToDraw.y + CurrentFrameToDraw.h) - 1 - getMaxAgeOfCovidBeacons();
	TFT_drawFastHLine( CurrentFrameToDraw.x + 1,
                       yActive,
					   CurrentFrameToDraw.w - 2,
					   TFT_DARKGREY);
	TFT_drawFastHLine( CurrentFrameToDraw.x + 1,
                       yMax,
					   CurrentFrameToDraw.w - 2,
					   TFT_DARKGREY);
}

void updateCovidBeaconHistoryCB( void )
{
	int n;
	for (n = 0; n < getNumberOfPossibleCovidBeacons(); n++ )
	{
		int currentXoffset = (n * 3) + 1;
		int x = pCurrentFieldToUpdate->xPosOfValue + currentXoffset;
		int h = getAgeOfCovidBeacon(n);
		int y = (CurrentFrameToDraw.y + CurrentFrameToDraw.h) - 1 - h;
		int ymax = (CurrentFrameToDraw.y + CurrentFrameToDraw.h) - 1 - getMaxAgeOfCovidBeacons();
	    color_t barColorValue = pCurrentFieldToUpdate->foregroundColorValue;
	    if (getExposureTimeOfCovidBeacon(n) > (5UL * 60UL))
	    {
	        barColorValue = pCurrentFieldToUpdate->highlightColorValue;
	    }

		if ( x < ( DEFAULT_TFT_DISPLAY_WIDTH - 1 ))
		{
			TFT_drawFastVLine( x,
		                       y,
							   h,
							   barColorValue);
			TFT_drawFastVLine( x,
		                       ymax,
							   y-ymax,
							   pCurrentFieldToUpdate->backgroundColor);
			x++;
			TFT_drawFastVLine( x,
		                       y,
							   h,
							   barColorValue);
			TFT_drawFastVLine( x,
		                       ymax,
							   y-ymax,
							   pCurrentFieldToUpdate->backgroundColor);

		}
	}
}


///////////////////////////////////////////////////////////////////////////////
framefield_t SysInfoFields[] = {
    {
        .xOffset = XFIELD3_1,
        .fieldLine = 1,
        .maxFieldWidth = FIELD3_WIDTH,
        .backgroundColor = _TFT_BLACK,
        .foregroundColorName = _TFT_WHITE,
        .foregroundColorValue = _TFT_GREEN,
        .szName = "free",
        .szInitialValue = "---",
        .nameFont = UBUNTU16_FONT,
        .valueFont = UBUNTU16_FONT,
        .updateFieldValueCB = updateFreeSizeFieldCB
    },
    {
        .xOffset = XFIELD3_2,
        .fieldLine = 1,
        .maxFieldWidth = FIELD3_WIDTH,
        .backgroundColor = _TFT_BLACK,
        .foregroundColorName = _TFT_WHITE,
        .foregroundColorValue = _TFT_GREEN,
        .szName = "largest",
        .szInitialValue = "---",
        .nameFont = UBUNTU16_FONT,
        .valueFont = UBUNTU16_FONT,
        .updateFieldValueCB = updateLargestFreeSizeFieldCB
    },
    {
        .xOffset = XFIELD3_3,
        .fieldLine = 1,
        .maxFieldWidth = FIELD3_WIDTH,
        .backgroundColor = _TFT_BLACK,
        .foregroundColorName = _TFT_WHITE,
        .foregroundColorValue = _TFT_YELLOW,
        .szName = "delta",
        .szInitialValue = "---",
        .nameFont = UBUNTU16_FONT,
        .valueFont = UBUNTU16_FONT,
        .updateFieldValueCB = updateFreeSizeDeltaFieldCB
    },
    {
        .fieldLine = 0 // end of field list
    }
};

frameDef_t SysInfoFrame = {
        .x = 0,
        .xOffsetTitle = 10,
        .y = YFRAME1,
        .w = DEFAULT_TFT_DISPLAY_WIDTH,
        .h = HFRAME1,
        .backgroundColor = _TFT_BLACK,
        .foregroundColor = _TFT_WHITE,
        .szTitle = " System Infos ",
        .titleFont = UBUNTU16_FONT,
        .frameFields = SysInfoFields,
        .initializeFrameCB = NULL
};

framefield_t CovidInfoFields[] = {
	{
		.xOffset = XFIELD4_1,
		.fieldLine = 1,
		.maxFieldWidth = FIELD4_WIDTH,
		.backgroundColor = _TFT_BLACK,
		.foregroundColorName = _TFT_WHITE,
		.foregroundColorValue = _TFT_CYAN,
		.szName = "Active",
		.szInitialValue = "",
		.nameFont = UBUNTU16_FONT,
		.valueFont = UBUNTU16_FONT,
		.updateFieldValueCB = _updateActiveBeaconNrFieldCB
	},
	{
		.xOffset = XFIELD4_2,
		.fieldLine = 1,
		.maxFieldWidth = FIELD4_WIDTH,
		.backgroundColor = _TFT_BLACK,
		.foregroundColorName = _TFT_WHITE,
		.foregroundColorValue = _TFT_CYAN,
		.szName = "Total",
		.szInitialValue = "",
		.nameFont = UBUNTU16_FONT,
		.valueFont = UBUNTU16_FONT,
		.updateFieldValueCB = _updateTotalBeaconNrFieldCB
	},
	{
		.xOffset = XFIELD4_3,
		.fieldLine = 1,
		.maxFieldWidth = FIELD4_WIDTH,
		.backgroundColor = _TFT_BLACK,
		.foregroundColorName = _TFT_WHITE,
		.foregroundColorValue = _TFT_CYAN,
		.szName = "Max",
		.szInitialValue = "",
		.nameFont = UBUNTU16_FONT,
		.valueFont = UBUNTU16_FONT,
		.updateFieldValueCB = _updateMaxCovidBeaconsCB
	},
	{
		.xOffset = XFIELD4_4,
		.fieldLine = 1,
		.maxFieldWidth = FIELD4_WIDTH,
		.backgroundColor = _TFT_BLACK,
		.foregroundColorName = _TFT_WHITE,
		.foregroundColorValue = _TFT_CYAN,
		.szName = "Sum",
		.szInitialValue = "",
		.nameFont = UBUNTU16_FONT,
		.valueFont = UBUNTU16_FONT,
		.updateFieldValueCB = _updateTotalSumOfBeaconsCB
	},
    {
        .xOffset = 1,
        .fieldLine = 1,
        .maxFieldWidth = DEFAULT_TFT_DISPLAY_WIDTH - 2,
        .backgroundColor = _TFT_BLACK,
        .foregroundColorName = _TFT_WHITE,
        .foregroundColorValue = _TFT_CYAN,
        .highlightColorValue = _TFT_MAGENTA,
        .szName = "",
        .szInitialValue = "",
        .nameFont = SMALL_FONT,
        .valueFont = SMALL_FONT,
        .updateFieldValueCB = updateCovidBeaconHistoryCB
    },
    {
        .fieldLine = 0, // end of field list
        .updateFieldValueCB = NULL
    }
};

frameDef_t CovidInfoFrame = {
        .x = 0,
        .xOffsetTitle = 10,
        .y = YFRAME2,
        .w = DEFAULT_TFT_DISPLAY_WIDTH,
        .h = HFRAME2,
        .backgroundColor = _TFT_BLACK,
        .foregroundColor = _TFT_WHITE,
        .szTitle = " CovidBeacons ",
        .titleFont = UBUNTU16_FONT,
        .frameFields = CovidInfoFields,
        .initializeFrameCB = initializeCovidBeaconHistoryCB
};

void _diag_task()
{
    static uint16_t backlightPWM = 4095;

    drawHeader();
    drawSoftKeys();
    updateClock();

    pwmSetDutyCycle(BL_PWM, backlightPWM);

    drawFrame( SysInfoFrame );
    drawFrame( CovidInfoFrame );

    for(;;) {
        uint8_t recv = ' ';
        if ( pdPASS == xQueueReceive ( hDiagCcontrolQueue,
                                        &recv,
                                        pdMS_TO_TICKS(500UL) ) )
        {
//            ESP_LOGI(__func__, "got %c", recv);
            switch ( recv )
            {
                case '!': // update infos
                    updateFrame( CovidInfoFrame );
                    break;
                default:
                    break;
            }
        }
        else
        {
            if ( ( ( (xTaskGetTickCount() * portTICK_PERIOD_MS)  / 1000) < 5 ) ||
                 ( gpio_get_level( M5_PIN_NUM_BTN_A ) == 0 ) )
            {
                FreeSpaceBaseline= heap_caps_get_free_size(MALLOC_CAP_8BIT);
            }
            if ( gpio_get_level( M5_PIN_NUM_BTN_B ) == 0 )
            {
            	if ( backlightPWM != 0 ) // display must be ON to obey the BTN_B
            	{
                	clrAllCovidBeacons();
                	updateFrame( CovidInfoFrame );
            	}
            }
			if ( gpio_get_level( M5_PIN_NUM_BTN_C ) == 0 )
            {
			    switch (backlightPWM)
			    {
			       case 0:
			           backlightPWM = 128;
			           break;
                   case 128:
                       backlightPWM = 4095;
                       break;
			       default:
                       backlightPWM = 0;
                       break;
			    }
			    pwmSetDutyCycle(BL_PWM, backlightPWM);
            }
            updateClock();
            updateFrame( SysInfoFrame );
        }
    }
}
