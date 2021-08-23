#include <stdio.h>

#define log_printf(macropar_message, ...) \
        printf(macropar_message, ##__VA_ARGS__)


/* default values for CO_CANopenInit() */
#define NMT_CONTROL \
            CO_NMT_STARTUP_TO_OPERATIONAL \
          | CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 501
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK true
#define OD_STATUS_BITS NULL

/* 72 / 8 => 9000000 counts per second,  */
#define SYSTICKS_PER_MS 9000

#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)

#include "FreeRTOS.h"
#include "CANopen.h"
#include "OD.h"
#include "CO_storageBlank.h"


/* Global variables and objects */
CO_t *CO = NULL; /* CANopen object */
uint8_t LED_red, LED_green;

static void gpio_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Configure LED GPIOs. */
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);


}

static void can_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);  // need to be turned on for remapping A to B
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
    gpio_set_mode(GPIOB,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,GPIO_CAN_PB_TX);
    gpio_set_mode(GPIOB,GPIO_MODE_INPUT,GPIO_CNF_INPUT_FLOAT,GPIO_CAN_PB_RX);
    
    gpio_primary_remap(                   // Map CAN1 to use PB8/PB9
        AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, // Optional
        AFIO_MAPR_CAN1_REMAP_PORTB);      // CAN_RX=PB8, CAN_TX=PB9
    
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);
}

static void systick_setup()
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(SYSTICKS_PER_MS - 1);
    

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

/* main ***********************************************************************/
int main (void){
    gpio_setup();
    can_setup();

    CO_ReturnError_t err;
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;
    uint32_t heapMemoryUsed;
    void *CANptr = NULL; /* CAN module address */
    uint8_t pendingNodeId = 4; /* read from dip switches or nonvolatile memory, configurable by LSS slave */
    uint8_t activeNodeId = 4; /* Copied from CO_pendingNodeId in the communication reset section */
    uint16_t pendingBitRate = 1000;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;
    CO_storage_entry_t storageEntries[] = {
        {
            .addr = &OD_PERSIST_COMM,
            .len = sizeof(OD_PERSIST_COMM),
            .subIndexOD = 2,
            .attr = CO_storage_cmd | CO_storage_restore,
            .addrNV = NULL
        }
    };
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    /* Configure microcontroller. */


    /* Allocate memory */
    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 0;
    }
    else {
        log_printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }


#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage,
                               CO->CANmodule,
                               OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters,
                               storageEntries,
                               storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", storageInitError);
        return 0;
    }
#endif


    while(reset != CO_RESET_APP){
/* CANopen communication reset - initialize CANopen objects *******************/
        log_printf("CANopenNode - Reset communication...\n");

        /* Wait rt_thread. */
        CO->CANmodule->CANnormal = false;

        /* Enter CAN configuration. */
        CO_CANsetConfigurationMode((void *)&CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        /* initialize CANopen */
        err = CO_CANinit(CO, CANptr, pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_printf("Error: CAN initialization failed: %d\n", err);
            return 0;
        }

        CO_LSS_address_t lssAddress = {.identity = {
            .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
            .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
            .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
            .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
        }};
        err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
        if(err != CO_ERROR_NO) {
            log_printf("Error: LSS slave initialization failed: %d\n", err);
            return 0;
        }

        activeNodeId = pendingNodeId;
        uint32_t errInfo = 0;

        err = CO_CANopenInit(CO,                /* CANopen object */
                             NULL,              /* alternate NMT */
                             NULL,              /* alternate em */
                             OD,                /* Object dictionary */
                             OD_STATUS_BITS,    /* Optional OD_statusBits */
                             NMT_CONTROL,       /* CO_NMT_control_t */
                             FIRST_HB_TIME,     /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,     /* SDOclientBlockTransfer */
                             activeNodeId,
                             &errInfo);
        if(err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            }
            else {
                log_printf("Error: CANopen initialization failed: %d\n", err);
            }
            return 0;
        }

        /* Configure Timer interrupt function for execution every 1 millisecond */
        systick_setup();

        /* Configure CAN transmit and receive interrupt */


        /* Configure CANopen callbacks, etc */
        if(!CO->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if(storageInitError != 0) {
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY,
                               CO_EMC_HARDWARE, storageInitError);
            }
#endif
        }
        else {
            log_printf("CANopenNode - Node-id not initialized\n");
        }


        /* start CAN */
        CO_CANsetNormalMode(CO->CANmodule);

        reset = CO_RESET_NOT;

        log_printf("CANopenNode - Running...\n");
        fflush(stdout);

        uint32_t last_tick = systick_get_value();
        while(reset == CO_RESET_NOT){
/* loop for normal program execution ******************************************/
            /* get time difference since last function call */
            
            /* CANopen process */
            uint32_t current_tick = systick_get_value();
            uint32_t time_difference_us = (((last_tick - current_tick) + SYSTICKS_PER_MS) % SYSTICKS_PER_MS) * 1000 / SYSTICKS_PER_MS;
            last_tick = current_tick;

            reset = CO_process(CO, false, time_difference_us, NULL);
            LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
            LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);


            /* Nonblocking application code may go here. */

            /* Process automatic storage */

            /* optional sleep for short time */
        }
    }


/* program exit ***************************************************************/
    /* stop threads */


    /* delete objects from memory */
    CO_CANsetConfigurationMode((void *)&CANptr);
    CO_delete(CO);

    log_printf("CANopenNode finished\n");

    /* reset */
    return 0;
}


/* timer thread executes in constant intervals ********************************/
void sys_tick_handler(void){
    interval_task();
    //for(;;) {
    //}
}

void interval_task(void) {
    CO_LOCK_OD(CO->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
        bool_t syncWas = false;
        /* get time difference since last function call */
        uint32_t timeDifference_us = 1000;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

        /* Further I/O or nonblocking application code may go here. */
    }
    CO_UNLOCK_OD(CO->CANmodule);
}


void usb_lp_can_rx0_isr(void)
{

	gpio_toggle(GPIOC, GPIO13);
    

    CO_CANinterrupt(CO->CANmodule, 0, CAN_RF0R(CO->CANmodule->CANport)&3);
    /* clear interrupt flag */
}

void usb_hp_can_tx_isr(void)
{
	gpio_toggle(GPIOC, GPIO13);
    CO_CANinterrupt(CO->CANmodule, 1, 0);
}

