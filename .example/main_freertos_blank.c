#include <stdio.h>

#define log_printf(macropar_message, ...) \
  printf(macropar_message, ##__VA_ARGS__)

/* default values for CO_CANopenInit() */
#define NMT_CONTROL             \
  CO_NMT_STARTUP_TO_OPERATIONAL \
  | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 501
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK true
#define OD_STATUS_BITS NULL
#define CO_CONFIG_SDO_SRV                                             \
  (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | \
   CO_CONFIG_GLOBAL_FLAG_TIMERNEXT | CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC)
#define INCLUDE_vTaskSuspend 1

#include "CANopen.h"
#include "CO_storageFlash.h"
#include "FreeRTOS.h"
#include "OD.h"
#include "task.h"

typedef struct {
  uint8_t nodeId; /* read from dip switches or nonvolatile memory, configurable
                     by LSS slave */
  uint16_t bitRate; /* read from dip switches or nonvolatile memory,
                       configurable by LSS slave */
} CO_config_communication_t;

/* Global variables and objects */
CO_t *CO = NULL;     /* CANopen object */
void *CANptr = NULL; /* CAN module address, user defined structure */

/* Initial values for*/
CO_config_communication_t CO_config_communication = {.nodeId = 4,
                                                     .bitRate = 1000};

/* List of values to be stored in flash storage */
CO_storage_t CO_storage;
CO_storage_entry_t storageEntries[] = {
    {.addr = &OD_PERSIST_COMM,
     .len = sizeof(OD_PERSIST_COMM),
     .subIndexOD = 2,
     .attr = CO_storage_cmd | CO_storage_restore},
    {.addr = &CO_config_communication,
     .len = sizeof(CO_config_communication_t),
     .attr = CO_storage_cmd | CO_storage_restore | CO_storage_auto}};
uint8_t storageEntriesCount =
    sizeof(storageEntries) / sizeof(storageEntries[0]);

uint8_t LED_red, LED_green;

TaskHandle_t TaskCOControlHandle;
TaskHandle_t TaskCOPeriodicalHandle;
TaskHandle_t TaskCOProcessingHandle;

static uint32_t get_time_difference_us(uint32_t *last_tick) {
  uint32_t current_tick = systick_get_value();
  uint32_t time_difference_us =
      (((*last_tick - current_tick) + SYSTICKS_PER_MS) % SYSTICKS_PER_MS) *
      1000 / SYSTICKS_PER_MS;
  *last_tick = current_tick;
  return time_difference_us;
}

static void configure_gpio(void) {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_GPIOA);

  /* Configure LED GPIOs. */
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO13);
}

static void configure_can(void) {
  rcc_periph_clock_enable(
      RCC_GPIOA);  // need to be turned on for remapping A to B
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                GPIO_CAN_PB_TX);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_CAN_PB_RX);

  gpio_primary_remap(                    // Map CAN1 to use PB8/PB9
      AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,  // Optional
      AFIO_MAPR_CAN1_REMAP_PORTB);       // CAN_RX=PB8, CAN_TX=PB9

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
  uint32_t heapMemoryUsed;
  /* Allocate memory */
  CO = CO_new(NULL, &heapMemoryUsed);
  if (CO == NULL) {
    log_printf("Error: Can't allocate memory\n");
    return CO_ERROR_OUT_OF_MEMORY;
  } else {
    log_printf("Allocated %u bytes for CANopen objects\n",
               (unsigned int)heapMemoryUsed);
  }
  return CO_ERROR_NO;
}

static CO_ReturnError_t initialize_storage(uint32_t *storageInitError) {
  log_printf("CANopenNode - Initialize storage...\n");

  CO_ReturnError_t err;

  err = CO_storageFlash_init(
      &CO_storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
      OD_ENTRY_H1011_restoreDefaultParameters, storageEntries,
      storageEntriesCount, storageInitError);

  if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
    log_printf("Error: Storage %d\n", (int)*storageInitError);
    return err;
  }

  return CO_ERROR_NO;
}

static CO_ReturnError_t initialize_communication(void) {
  CO_ReturnError_t err;

  log_printf("CANopenNode - Reset communication...\n");
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
  CO_LSS_address_t lssAddress = {
      .identity = {
          .vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
          .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
          .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
          .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};

  err = CO_LSSinit(CO, &lssAddress, &CO_config_communication.nodeId,
                   &CO_config_communication.bitRate);
  if (err != CO_ERROR_NO) {
    log_printf("Error: LSS slave initialization failed: %d\n", err);
    return err;
  }

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
      log_printf("Error: Object Dictionary entry 0x%X\n",
                 (unsigned int)errInfo);
    } else {
      log_printf("Error: CANopen initialization failed: %d\n", err);
    }
    return err;
  }

  /* Configure CANopen callbacks, etc */
  if (!CO->nodeIdUnconfigured) {
    if (storageInitError != 0) {
      CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE,
                     storageInitError);
    }
  } else {
    log_printf("CANopenNode - Node-id not initialized\n");
  }

  /* start CAN */
  CO_CANsetNormalMode(CO->CANmodule);

  return CO_ERROR_NO;
}

/* Task that configures devices and handles NMT commands */
static void TaskCOControl(void * pvParameters) {
	(void) pvParameters; /* unused */
  uint32_t reset = CO_RESET_COMM;

  // Initial device setup
  configure_gpio();
  configure_can();
  configure_systick();

  for (;;) {
    fflush(stdout);
    // Full device reset
    if (reset >= CO_RESET_COMM) {
      CO_CANsetConfigurationMode((void *)&CANptr);
      CO_delete(CO);
    }

    // Reset communications: storage, canopen, communication
    if (reset == CO_RESET_COMM) {
      log_printf("CANopenNode - Resetting communication...\n");
      uint32_t storageInitError = 0;
      if (initialize_memory() != CO_ERROR_NO ||
          initialize_storage(&storageInitError) != CO_ERROR_NO ||
          initialize_communication() != CO_ERROR_NO ||
          initialize_canopen(storageInitError) != CO_ERROR_NO) {
        reset = CO_RESET_APP;
      }
    }

    if (reset >= CO_RESET_APP) {
      log_printf("CANopenNode finished\n");
      scb_reset_system();
    }

    // Wait for reset notification
    log_printf("CANopenNode - Running...\n");
    xTaskNotifyWaitIndexed(
        0,              /* Wait for 0th notification. */
        0x00,           /* Don't clear any notification bits on entry. */
        0x00,           /* Reset the notification value to 0 on exit. */
        &reset,         /* Retrieve reset command */
        portMAX_DELAY); /* Block indefinitely. */
  }
}

/* Main task that does slower work */
static void TaskCOProcessing(void * pvParameters) {
	(void) pvParameters; /* unused */
  uint32_t last_tick = systick_get_value();
  while (true) {
    uint32_t time_difference_us = get_time_difference_us(&last_tick);

    CO_NMT_reset_cmd_t reset = CO_process(CO, false, time_difference_us, NULL);

    if (reset != CO_RESET_NOT) {
      xTaskNotifyIndexed(TaskCOControlHandle, 0, reset, eSetValueWithOverwrite);
    }
    taskYIELD();
  }
}

/* Less frequent task that is invoked periodically */
static void TaskCOPeriodical(void * pvParameters) {
	(void) pvParameters; /* unused */
  uint16_t ticks_count = 0;
  uint32_t last_tick = systick_get_value();
  while (true) {
    uint32_t time_difference_us = get_time_difference_us(&last_tick);

    // Run basic tasks every ~1ms
    CO_LOCK_OD(co->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
      bool_t syncWas = false;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
      syncWas = CO_process_SYNC(CO, time_difference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
      CO_process_RPDO(CO, syncWas, time_difference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
      CO_process_TPDO(CO, syncWas, time_difference_us, NULL);
#endif
    }
    CO_UNLOCK_OD(co->CANmodule);
  
    LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
    LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

    // wait for 1ms
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Run storage routine every 1000mss */
    if (ticks_count++ > 1000) {
      ticks_count = 0;
      CO_storageFlash_auto_process(&CO_storage);
    }
  }
}

/* main ***********************************************************************/
int main(void) {
  xTaskCreate(TaskCOControl, "COControl", 100, NULL, 3,
              &TaskCOControlHandle);
  xTaskCreate(TaskCOPeriodical, "COPeriodical", 50, NULL,
              2, &TaskCOPeriodicalHandle);
  xTaskCreate(TaskCOProcessing, "COProcessing", 50, NULL,
             1, &TaskCOProcessingHandle);

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