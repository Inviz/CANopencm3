
/* semihosting Initializing */
extern void initialise_monitor_handles(void);

#define log_printf(macropar_message, ...) printf(macropar_message, ##__VA_ARGS__)

/* default values for CO_CANopenInit() */
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 501
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK true
#define OD_STATUS_BITS NULL

#define CO_GET_CO(obj) CO_##obj
#define CO_GET_CNT(obj) OD_CNT_##obj


#ifdef SEMIHOSTING
  #include <stdio.h>
#endif

#include "CANopen.h"

#include "CO_storageFlash.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "OD.h"
#include "task.h"

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName ) {
    (void) xTask; /* unused*/
    printf("System - Stack overflow! %s", pcTaskName);
}

typedef struct {
    uint16_t nodeId;  /* read from dip switches or nonvolatile memory, configurable
                        by LSS slave */
    uint16_t bitRate; /* read from dip switches or nonvolatile memory,
                         configurable by LSS slave */
} CO_config_communication_t;

/* Global variables and objects */
CO_t *CO = NULL;     /* CANopen object */
void *CANptr = NULL; /* CAN module address, user defined structure */

/* Initial values for*/
CO_config_communication_t CO_config_communication = {.nodeId = 4, .bitRate = 1000};

/* List of values to be stored in flash storage */
CO_storage_t CO_storage;
CO_storage_entry_t storageEntries[] = {
    /* Entirety of persistent section OD */
    {.addr = &OD_PERSIST_COMM,
     .len = sizeof(OD_PERSIST_COMM),
     .subIndexOD = 2,
     .attr = CO_storage_cmd | CO_storage_restore},

    /* Negotiated LSS settings */
    {.addr = &CO_config_communication,
     .len = sizeof(CO_config_communication_t),
     .subIndexOD = 0,
     .attr = CO_storage_cmd | CO_storage_restore | CO_storage_auto}};
uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);

uint8_t LED_red, LED_green;

TaskHandle_t TaskCANOpenControlHandle;
TaskHandle_t TaskCANOpenProcessingHandle;
TaskHandle_t TaskCANOpenMainlineHandle;



SemaphoreHandle_t TaskMainlineSemaphore = NULL;
SemaphoreHandle_t TaskProcessingSemaphore = NULL;


static void configure_gpio(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Configure LED GPIOs. */
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

static void configure_can(void) {
    rcc_periph_clock_enable(RCC_GPIOA); // need to be turned on for remapping A to B
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_CAN_PB_TX);
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_CAN_PB_RX);

    gpio_primary_remap(                   // Map CAN1 to use PB8/PB9
        AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, // Optional
        AFIO_MAPR_CAN1_REMAP_PORTB);      // CAN_RX=PB8, CAN_TX=PB9

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1);
}

/* Used to track time precisely */
static void configure_systick(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(SYSTICKS_PER_MS - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

static CO_ReturnError_t initialize_memory(void) {
    uint32_t heapMemoryUsed = 0;
    /* Allocate memory */
    CO = CO_new(NULL, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return CO_ERROR_OUT_OF_MEMORY;
    } else {
        if (heapMemoryUsed == 0) {
            log_printf("Config - Static memory\n", (unsigned int)heapMemoryUsed);
        } else {
            log_printf("Config - On heap (%ubytes)\n", (unsigned int)heapMemoryUsed);
        }
    }
    return CO_ERROR_NO;
}

static CO_ReturnError_t initialize_storage(uint32_t *storageInitError) {
    log_printf("Config - Storage...\n");

    CO_ReturnError_t err;

    err = CO_storageFlash_init(&CO_storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", (int)*storageInitError);
        return err;
    }

    return CO_ERROR_NO;
}


bool_t LSSStoreConfigCallback(void *object, uint8_t id, uint16_t bitRate) {
  log_printf("Config - Store LSS #%i @ %ikbps...\n", id, bitRate);
  CO_config_communication.bitRate = bitRate;
  CO_config_communication.nodeId = id;
  return CO_flash_write_entry(&storageEntries[1]) == CO_ERROR_NO;
}

static CO_ReturnError_t initialize_communication(void) {
    CO_ReturnError_t err;

    log_printf("Config - Communication...\n");
    /* Enter CAN configuration. */
    CO->CANmodule->CANnormal = false;
    CO_CANsetConfigurationMode((void *)&CANptr);
    CO_CANmodule_disable(CO->CANmodule);

    /* Initialize CANopen driver */
    err = CO_CANinit(CO, CANptr, CO_config_communication.bitRate);
    if (err != CO_ERROR_NO) {
        log_printf("Error: CAN initialization failed: %d\n", err);
        return err;
    }

    /* Engage LSS configuration */
    CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};

    err = CO_LSSinit(CO, &lssAddress, &CO_config_communication.nodeId, &CO_config_communication.bitRate);
    CO_LSSslave_initCfgStoreCallback(CO->LSSslave, NULL, LSSStoreConfigCallback);

    if (err != CO_ERROR_NO) {
        log_printf("Error: LSS slave initialization failed: %d\n", err);
        return err;
    }

    return CO_ERROR_NO;
}

static void TaskCANOpenMainlineCallback(void* object) { 
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( TaskMainlineSemaphore, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void TaskCANOpenProcessingCallback(void* object) {
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( TaskProcessingSemaphore, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static CO_ReturnError_t initialize_callbacks(CO_t *co) {
  /* Mainline tasks */
    if (CO_GET_CNT(EM) == 1) {
        CO_EM_initCallbackPre(co->em, NULL, TaskCANOpenMainlineCallback);
    }
    if (CO_GET_CNT(NMT) == 1) {
        CO_NMT_initCallbackPre(co->NMT, NULL, TaskCANOpenMainlineCallback);
    }
#if (CO_CONFIG_SRDO) & CO_CONFIG_SRDO_SRV_ENABLE
    for (int16_t i = 0; i < CO_GET_CNT(SRDO); i++) {
        CO_SRDO_initCallbackPre(co->SRDO[i], NULL, TaskCANOpenMainlineCallback);
    }
#endif
#if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
    if (CO_GET_CNT(HB_CONS) == 1) {
        CO_HBconsumer_initCallbackPre(co->HBCons, NULL, TaskCANOpenMainlineCallback);
    }
#endif
#if (CO_CONFIG_TIME) & CO_CONFIG_TIME_ENABLE
    if (CO_GET_CNT(TIME) == 1) {
        CO_TIME_initCallbackPre(co->TIME, NULL, TaskCANOpenMainlineCallback);
    }
#endif
#if (CO_CONFIG_SDO_CLI) & CO_CONFIG_SDO_CLI_ENABLE
    for (int16_t i = 0; i < CO_GET_CNT(SDO_CLI); i++) {
        CO_SDOclient_initCallbackPre(&co->SDOclient[i], NULL, TaskCANOpenMainlineCallback);
    }
#endif
for (int16_t i = 0; i < CO_GET_CNT(SDO_SRV); i++) {
    CO_SDOserver_initCallbackPre(&co->SDOserver[i], NULL, TaskCANOpenMainlineCallback);
}
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_MASTER
    CO_LSSmaster_initCallbackPre(co->LSSmaster, NULL, TaskCANOpenMainlineCallback);
#endif
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE
    CO_LSSslave_initCallbackPre(co->LSSslave, NULL, TaskCANOpenMainlineCallback);
#endif

  /* Processing tasks */
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
    if (CO_GET_CNT(SYNC) == 1) {
        CO_SYNC_initCallbackPre(co->SYNC, NULL, TaskCANOpenProcessingCallback);
    }
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
    for (int i = 0; i < CO_NO_RPDO; i++) {
        CO_RPDO_initCallbackPre(co->RPDO[i], (void *)0x01, TaskCANOpenProcessingCallback);
    }
#endif
    return CO_ERROR_NO;
}

static CO_ReturnError_t initialize_canopen(uint32_t storageInitError) {
    CO_ReturnError_t err;
    uint32_t errInfo = 0;

    err = CO_CANopenInit(CO,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBit
                                                */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         CO_config_communication.nodeId, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
        if (err == CO_ERROR_OD_PARAMETERS) {
            log_printf("Error: Object Dictionary entry 0x%X\n", (unsigned int)errInfo);
        } else {
            log_printf("Error: CANopen initialization failed: %d\n", err);
        }
        return err;
    }

    /* Configure CANopen callbacks, etc */
    if (!CO->nodeIdUnconfigured) {
        if (storageInitError != 0) {
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
        }
    } else {
        log_printf("CANopenNode - Node-id not initialized\n");
    }

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule);

    return CO_ERROR_NO;
}

/* Task that configures devices and handles NMT commands */
static void TaskCANOpenControl(void *pvParameters) {
    (void)pvParameters; /* unused */
    uint32_t reset = CO_RESET_COMM;

    log_printf("Config - Device settings...\n");
    // Initial device setup
    configure_gpio();
    configure_can();
    configure_systick();

    for (;;) {
        fflush(stdout);
        log_printf("System - Reset sequence %i...\n", CO_RESET_COMM);

        // Full device reset
        if (reset >= CO_RESET_COMM && CO != NULL) {
            log_printf("Config - Unloading...\n");
            CO_CANsetConfigurationMode((void *)&CANptr);
            CO_delete(CO);
        }

        // Reset communications: storage, canopen, communication
        if (reset == CO_RESET_COMM) {
            uint32_t storageInitError = 0;
            if (initialize_memory() != CO_ERROR_NO ||                   /* Allocate memory*/
                initialize_storage(&storageInitError) != CO_ERROR_NO || /* Read up the storage */
                initialize_communication() != CO_ERROR_NO ||            /* Set up LSS */
                initialize_canopen(storageInitError) != CO_ERROR_NO ||  /* Start canopen*/
                initialize_callbacks(CO) != CO_ERROR_NO) {               /* Subscribe to events*/
                reset = CO_RESET_APP;
            }
            log_printf("         #%i @ %ikbps\n", CO_config_communication.nodeId, CO_config_communication.bitRate);
        }

        if (reset >= CO_RESET_APP) {
            printf("System - Resetting...\n");
            scb_reset_system();
        }

        log_printf("System - Running...\n");
        // Wait for reset notification
        xTaskNotifyWaitIndexed(0,              /* Wait for 0th notification. */
                               0xffffffffUL,   /* Clear any notification bits on entry. */
                               0xffffffffUL,           /* Reset the notification value to 0 on exit. */
                               &reset,         /* Retrieve reset command */
                               portMAX_DELAY); /* Block indefinitely. */
    }
}

/* Main task that does slower work */
static void TaskCANOpenMainline(void *pvParameters) {
    (void)pvParameters; /* unused */
    TaskMainlineSemaphore = xSemaphoreCreateBinary();
    TickType_t last_tick = xTaskGetTickCount();
    while (true) {
        TickType_t current_tick = xTaskGetTickCount();
        uint32_t timeout = 0xffffffffUL;

        CO_NMT_reset_cmd_t reset = CO_process(CO, false, (current_tick - last_tick) * US_PER_TICK, &timeout);

        LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
        LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

        if (reset != CO_RESET_NOT) {
            if (xTaskNotifyIndexed(TaskCANOpenControlHandle, 0, reset, eSetValueWithOverwrite) == pdTRUE) {
              taskYIELD();
            }
        }

        last_tick = current_tick;
        xSemaphoreTake( TaskMainlineSemaphore, timeout / US_PER_TICK);
    }
}

/* Less frequent task that is invoked periodically */
static void TaskCANOpenProcessing(void *pvParameters) {
    (void)pvParameters; /* unused */
    TaskProcessingSemaphore = xSemaphoreCreateBinary();
    TickType_t last_tick = xTaskGetTickCount();
    TickType_t last_storage_tick = last_tick;
    while (true) {
        TickType_t current_tick = xTaskGetTickCount();
        uint32_t timeout = 2000000000;

        CO_LOCK_OD(co->CANmodule);
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, (current_tick - last_tick) * US_PER_TICK, &timeout);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, (current_tick - last_tick) * US_PER_TICK, &timeout);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, (current_tick - last_tick) * US_PER_TICK, &timeout);
#endif
        }
        CO_UNLOCK_OD(co->CANmodule);

        /* Thorttle autostorage to 1s */
        if ((current_tick - last_storage_tick) * US_PER_TICK > 1000000) {
            last_storage_tick = current_tick;
            log_printf("CANopenNode - Autosaving...\n");
            CO_storageFlash_auto_process(&CO_storage);
        }

        last_tick = current_tick;
        xSemaphoreTake( TaskProcessingSemaphore, timeout / US_PER_TICK );
    }
}
/* main ***********************************************************************/
int main(void) {
#ifdef SEMIHOSTING
    initialise_monitor_handles(); /* This Function MUST come before the first printf() */
    printf("System - Starting...\n");
#endif
    xTaskCreate(TaskCANOpenControl, "COControl", 200, NULL, 3, &TaskCANOpenControlHandle);
    xTaskCreate(TaskCANOpenMainline, "COMainline", 150, NULL, 2, &TaskCANOpenMainlineHandle);
    xTaskCreate(TaskCANOpenProcessing, "COProcessing", 50, NULL, 1, &TaskCANOpenProcessingHandle);
    
    printf("System - Starting tasks...\n");
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}

void usb_lp_can_rx0_isr(void) {
    gpio_toggle(GPIOC, GPIO13);
    CO_CANRxInterrupt(CO->CANmodule);
}

void usb_hp_can_tx_isr(void) {
    gpio_toggle(GPIOC, GPIO13);
    CO_CANTxInterrupt(CO->CANmodule);
}