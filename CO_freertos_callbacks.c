/*
static void ProcessTaskSignal(void) { xTaskNotifyIndexed(TaskCOProcessHandle, 0, 1, eSetValueWithOverwrite); }
static void PeriodicalTaskSignal(void) { xTaskNotifyIndexed(TaskCOProcessHandle, 0, 1, eSetValueWithOverwrite); }


static CO_ReturnError_t initialize_threads(CO_t *co) {
    if (CO_GET_CNT(EM) == 1) {
        CO_EM_initCallbackPre(co->em, (void *) 0x01, ProcessTaskSignal);
    }
    if (CO_GET_CNT(NMT) == 1) {
        CO_NMT_initCallbackPre(co->NMT, (void *) 0x02, PeriodicalTaskSignal);
    }
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
    if (CO_GET_CNT(SYNC) == 1) {
        CO_SYNC_initCallbackPre(co->SYNC, (void *) 0x03, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_HB_CONS) & CO_CONFIG_HB_CONS_ENABLE
    if (CO_GET_CNT(HB_CONS) == 1) {
        CO_HBconsumer_initCallbackPre(co->HBCons, (void *) 0x04, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_TIME) & CO_CONFIG_TIME_ENABLE
    if (CO_GET_CNT(TIME) == 1) {
        CO_TIME_initCallbackPre(co->TIME, NULL, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_SDO_CLI) & CO_CONFIG_SDO_CLI_ENABLE
    for (int16_t i = 0; i < CO_GET_CNT(SDO_CLI); i++) {
        CO_SDOclient_initCallbackPre(co->SDOclient[i], NULL, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_SDO_SRV) & CO_CONFIG_SDO_SRV_ENABLE
    for (int16_t i = 0; i < CO_GET_CNT(SDO_SRV); i++) {
        CO_SDOserver_initCallbackPre(co->SDOserver[i], NULL, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
    for (int i = 0; i < CO_NO_RPDO; i++) {
        CO_RPDO_initCallbackPre(co->RPDO[i], (void *) 0x01, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_SRDO) & CO_CONFIG_SRDO_SRV_ENABLE
    for (int16_t i = 0; i < CO_GET_CNT(SRDO); i++) {
      CO_SRDO_initCallbackPre(co->SRDO[i], NULL, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_MASTER
    if (CO_GET_CNT(LSS_MST) == 1) {
        CO_LSSmaster_initCallbackPre(co->LSSmaster, NULL, PeriodicalTaskSignal);
    }
#endif
#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE
    if (CO_GET_CNT(LSS_SLV) == 1) {
        CO_LSSslave_initCallbackPre(co->LSSslave, NULL, PeriodicalTaskSignal);
    }
#endif
}*/