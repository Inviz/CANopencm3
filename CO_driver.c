/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "301/CO_driver.h"

//#include <libopencm3/stm32/flash.h>



#ifndef CO_CANbitRateDataInitializers
    /* Macros, which divides K into (SJW + PROP + PhSeg1 + PhSeg2) */
    #define TQ_x_7    1, 2, 3, 1
    #define TQ_x_8    1, 2, 3, 2
    #define TQ_x_9    1, 2, 4, 2
    #define TQ_x_10   1, 3, 4, 2
    #define TQ_x_12   1, 3, 6, 2
    #define TQ_x_14   1, 4, 7, 2
    #define TQ_x_15   1, 4, 8, 2  /* good timing */
    #define TQ_x_16   1, 5, 8, 2  /* good timing */
    #define TQ_x_17   1, 6, 8, 2  /* good timing */
    #define TQ_x_18   1, 7, 8, 2  /* good timing */
    #define TQ_x_19   1, 8, 8, 2  /* good timing */
    #define TQ_x_20   1, 8, 8, 3  /* good timing */
    #define TQ_x_21   1, 8, 8, 4
    #define TQ_x_22   1, 8, 8, 5
    #define TQ_x_23   1, 8, 8, 6
    #define TQ_x_24   1, 8, 8, 7
    #define TQ_x_25   1, 8, 8, 8

    #if CO_FSYS == 4000
        #define CO_CANbitRateDataInitializers  \
        {10,  TQ_x_20, 10}, \
        {5,   TQ_x_20, 20}, \
        {2,   TQ_x_20, 50}, \
        {1,   TQ_x_16, 125}, \
        {1,   TQ_x_8 , 250}, \
        {1,   TQ_x_8 , 0}, \
        {1,   TQ_x_8 , 0}, \
        {1,   TQ_x_8 , 0}
    #elif CO_FSYS == 8000
        #define CO_CANbitRateDataInitializers  \
        {25,  TQ_x_16, 10}, \
        {10,  TQ_x_20, 20}, \
        {5,   TQ_x_16, 50}, \
        {2,   TQ_x_16, 125}, \
        {1,   TQ_x_16, 250}, \
        {1,   TQ_x_8 , 500}, \
        {1,   TQ_x_8 , 0}, \
        {1,   TQ_x_8 , 0}
    #elif CO_FSYS == 12000
        #define CO_CANbitRateDataInitializers  \
        {40,  TQ_x_15, 10}, \
        {20,  TQ_x_15, 20}, \
        {8,   TQ_x_15, 50}, \
        {3,   TQ_x_16, 125}, \
        {2,   TQ_x_12, 250}, \
        {1,   TQ_x_12, 500}, \
        {1,   TQ_x_12, 0}, \
        {1,   TQ_x_12, 0}
    #elif CO_FSYS == 16000
        #define CO_CANbitRateDataInitializers  \
        {50,  TQ_x_16, 10}, \
        {25,  TQ_x_16, 20}, \
        {10,  TQ_x_16, 50}, \
        {4,   TQ_x_16, 125}, \
        {2,   TQ_x_16, 250}, \
        {1,   TQ_x_16, 500}, \
        {1,   TQ_x_10, 800}, \
        {1,   TQ_x_8 , 1000}
    #elif CO_FSYS == 20000
        #define CO_CANbitRateDataInitializers  \
        {50,  TQ_x_20, 10}, \
        {25,  TQ_x_20, 20}, \
        {10,  TQ_x_20, 50}, \
        {5,   TQ_x_16, 125}, \
        {2,   TQ_x_20, 250}, \
        {1,   TQ_x_20, 500}, \
        {1,   TQ_x_20, 0}, \
        {1,   TQ_x_10, 1000}
    #elif CO_FSYS == 24000
        #define CO_CANbitRateDataInitializers  \
        {63,  TQ_x_19, 10}, \
        {40,  TQ_x_15, 20}, \
        {15,  TQ_x_16, 50}, \
        {6,   TQ_x_16, 125}, \
        {3,   TQ_x_16, 250}, \
        {2,   TQ_x_12, 500}, \
        {1,   TQ_x_15, 800}, \
        {1,   TQ_x_12, 1000}
    #elif CO_FSYS == 32000
        #define CO_CANbitRateDataInitializers  \
        {64,  TQ_x_25, 10}, \
        {50,  TQ_x_16, 20}, \
        {20,  TQ_x_16, 50}, \
        {8,   TQ_x_16, 125}, \
        {4,   TQ_x_16, 250}, \
        {2,   TQ_x_16, 500}, \
        {2,   TQ_x_10, 800}, \
        {1,   TQ_x_16, 1000}
    #elif CO_FSYS == 36000
        #define CO_CANbitRateDataInitializers  \
        {50,  TQ_x_18, 10}, \
        {50,  TQ_x_18, 20}, \
        {20,  TQ_x_18, 50}, \
        {8,   TQ_x_18, 125}, \
        {4,   TQ_x_18, 250}, \
        {2,   TQ_x_18, 500}, \
        {2,   TQ_x_18, 0}, \
        {1,   TQ_x_18, 1000}
    #elif CO_FSYS == 40000
        #define CO_CANbitRateDataInitializers  \
        {50,  TQ_x_20, 0}, \
        {50,  TQ_x_20, 20}, \
        {25,  TQ_x_16, 50}, \
        {10,  TQ_x_16, 125}, \
        {5,   TQ_x_16, 250}, \
        {2,   TQ_x_20, 500}, \
        {1,   TQ_x_25, 800}, \
        {1,   TQ_x_20, 1000}
    #elif CO_FSYS == 48000
        #define CO_CANbitRateDataInitializers  \
        {63,  TQ_x_19, 0}, \
        {63,  TQ_x_19, 20}, \
        {30,  TQ_x_16, 50}, \
        {12,  TQ_x_16, 125}, \
        {6,   TQ_x_16, 250}, \
        {3,   TQ_x_16, 500}, \
        {2,   TQ_x_15, 800}, \
        {2,   TQ_x_12, 1000}
    #elif CO_FSYS == 56000
        #define CO_CANbitRateDataInitializers  \
        {61,  TQ_x_23, 0}, \
        {61,  TQ_x_23, 20}, \
        {35,  TQ_x_16, 50}, \
        {14,  TQ_x_16, 125}, \
        {7,   TQ_x_16, 250}, \
        {4,   TQ_x_14, 500}, \
        {5,   TQ_x_7 , 800}, \
        {2,   TQ_x_14, 1000}
    #elif CO_FSYS == 64000
        #define CO_CANbitRateDataInitializers  \
        {64,  TQ_x_25, 0}, \
        {64,  TQ_x_25, 20}, \
        {40,  TQ_x_16, 50}, \
        {16,  TQ_x_16, 125}, \
        {8,   TQ_x_16, 250}, \
        {4,   TQ_x_16, 500}, \
        {2,   TQ_x_20, 800}, \
        {2,   TQ_x_16, 1000}
    #elif CO_FSYS == 72000
        #define CO_CANbitRateDataInitializers  \
        {40,  TQ_x_18, 0}, \
        {40,  TQ_x_18, 0}, \
        {40,  TQ_x_18, 50}, \
        {16,  TQ_x_18, 125}, \
        {8,   TQ_x_18, 250}, \
        {4,   TQ_x_18, 500}, \
        {3,   TQ_x_15, 800}, \
        {2,   TQ_x_18, 1000}
    #elif CO_FSYS == 80000
        #define CO_CANbitRateDataInitializers  \
        {40,  TQ_x_20, 0}, \
        {40,  TQ_x_20, 0}, \
        {40,  TQ_x_20, 50}, \
        {16,  TQ_x_20, 125}, \
        {8,   TQ_x_20, 250}, \
        {4,   TQ_x_20, 500}, \
        {2,   TQ_x_25, 800}, \
        {2,   TQ_x_20, 1000}
    #else
        #error define_CO_FSYS CO_FSYS not supported
    #endif
#endif /* CO_CANbitRateDataInitializers */

typedef struct {
    uint8_t   BRP;      /* (1...64) Baud Rate Prescaler */
    uint8_t   SJW;      /* (1...4) SJW time */
    uint8_t   PROP;     /* (1...8) PROP time */
    uint8_t   phSeg1;   /* (1...8) Phase Segment 1 time */
    uint8_t   phSeg2;   /* (1...8) Phase Segment 2 time */
    uint16_t  bitrate;  /* bitrate in kbps */
} CO_CANbitRateData_t;

const CO_CANbitRateData_t CO_CANbitRateData[] = {
    CO_CANbitRateDataInitializers
};

bool_t CO_LSSchkBitrateCallback(void *object, uint16_t bitRate) {
    (void)object;
    uint32_t i;

    for (i=0; i<(sizeof(CO_CANbitRateData)/sizeof(CO_CANbitRateData[0])); i++) {
        if (CO_CANbitRateData[i].bitrate == bitRate && bitRate > 0) {
            return true;
        }
    }
    return false;
}


/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    (void) CANptr; /* unused */
    can_reset(CO_CAN_INTERFACE);
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */

    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;
    const CO_CANbitRateData_t *CANbitRateData = NULL;


    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;//(rxSize <= 32U) ? true : false;/* microcontroller dependent */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
    }


    /* Configure CAN module registers */
    CANmodule->CANport = CO_CAN_INTERFACE;
    CANmodule->CANrxFifoIndex = CO_CAN_RX_FIFO_INDEX;

    /* Configure CAN timing */
    for (i=0; i<(sizeof(CO_CANbitRateData)/sizeof(CO_CANbitRateData[0])); i++) {
        if (CO_CANbitRateData[i].bitrate == CANbitRate) {
            CANbitRateData = &CO_CANbitRateData[i];
            break;
        }
    }
    if (CANbitRate == 0 || CANbitRateData == NULL) {
        return CO_ERROR_ILLEGAL_BAUDRATE;
    }
    if (can_init(CANmodule->CANport,
            false,           /* TTCM: Time triggered comm mode? */
            true,            /* ABOM: Automatic bus-off management? */
            true,            /* AWUM: Automatic wakeup mode? */
            false,           /* NART: No automatic retransmission? */
            false,           /* RFLM: Receive FIFO locked mode? */
            false,           /* TXFP: Transmit FIFO priority? */
            (CANbitRateData->SJW - 1) << CAN_BTR_SJW_SHIFT,
            (CANbitRateData->phSeg1 + CANbitRateData->PROP - 1) << CAN_BTR_TS1_SHIFT,
            (CANbitRateData->phSeg2 - 1) << CAN_BTR_TS2_SHIFT,
            CANbitRateData->BRP,
            false,
            false) != 0) {
        return CO_ERROR_INVALID_STATE;
    }

	/* CAN filter 0 init. */
	can_filter_id_mask_32bit_init(
				0,     /* Filter ID */
				0,     /* CAN ID */
				0,     /* CAN ID mask */
				0,     /* FIFO assignment (here: FIFO0) */
				true); /* Enable the filter. */
                
    can_enable_irq(CAN1, CAN_IER_FMPIE0);   
    /* Configure CAN module hardware filters */
    if(CANmodule->useCANrxFilters){
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received message must match filter */
    }
    else{
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        /* Configure mask 0 so, that all messages with standard identifier are accepted */
    }


    /* configure CAN interrupt registers */


    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
    if (CANmodule != NULL) {
        CO_CANsetConfigurationMode(CANmodule->CANptr);
    }
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident & 0x07FFU;
        if(rtr){
            buffer->ident |= 0x0800U;
        }
        buffer->mask = (mask & 0x07FFU) | 0x0800U;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters) {
        //    can_filter_id_mask_32bit_init(
        //        index,     /* Filter ID */
        //        CANmodule->CANport,     /* CAN ID */
        //        buffer->ident,     /* CAN ID mask */
        //        buffer->mask,     /* FIFO assignment (here: FIFO0) */
        //        true); /* Enable the filter. */
        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = ((uint32_t)ident & 0x07FFU)
                      | ((uint32_t)(((uint32_t)noOfBytes & 0xFU) << 12U))
                      | ((uint32_t)(rtr ? 0x8000U : 0U));

        buffer->DLC = noOfBytes;
        buffer->rtr = rtr;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    /* Try sending message right away, if any of mailboxes are available */
    if(can_transmit(CANmodule->CANport, buffer->ident, false, buffer->rtr, buffer->DLC, buffer->data) != -1){
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    }
    /* if no buffer is free, message will be sent by interrupt */
    else{
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
        can_enable_irq(CANmodule->CANport, CAN_IER_TMEIE);
    }
    
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);


    if(tpdoDeleted != 0U){
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
    * different way to determine errors. */
static uint16_t rxErrors=0, txErrors=0, overflow=0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {
    uint32_t err;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors >= 256U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        if (overflow != 0) {
            /* CAN RX bus overflow */
            status |= CO_CAN_ERRRX_OVERFLOW;
        }

        CANmodule->CANerrorStatus = status;
    }
}


/******************************************************************************/
void CO_CANinterrupt(CO_CANmodule_t *CANmodule, uint8_t reason, uint8_t msgcount){

    /* receive interrupt */
    if(reason == 0){
        while ( msgcount-- > 0 ) {
            bool ext, rtr;
            uint8_t fmi;
            
            CO_CANrxMsg_t rcvMsg;      /* pointer to received message in CAN module */
            uint16_t index;             /* index of received message */
            CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
            bool_t msgMatched = false;


            can_receive(CANmodule->CANport, CANmodule->CANrxFifoIndex, true, &rcvMsg.ident, &ext, &rtr, &fmi, &rcvMsg.DLC, rcvMsg.data, NULL);

            if(CANmodule->useCANrxFilters){
                /* CAN module filters are used. Message with known 11-bit identifier has */
                /* been received */
                index = 0;  /* get index of the received message here. Or something similar */
                if(index < CANmodule->rxSize){
                    buffer = &CANmodule->rxArray[index];
                    /* verify also RTR */
                    if(((rcvMsg.ident ^ buffer->ident) & buffer->mask) == 0U){
                        msgMatched = true;
                    }
                }
            }
            else{
                /* CAN module filters are not used, message with any standard 11-bit identifier */
                /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
                buffer = &CANmodule->rxArray[0];
                for(index = CANmodule->rxSize; index > 0U; index--){
                    if(((rcvMsg.ident ^ buffer->ident) & buffer->mask) == 0U){
                        msgMatched = true;
                        break;
                    }
                    buffer++;
                }
            }

            /* Call specific function, which will process the message */
            if(msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)){
                buffer->CANrx_callback(buffer->object, &rcvMsg);
            }
        }

        /* Clear interrupt flag */
    }


    /* transmit interrupt */
    else if(reason == 1){
        /* Clear interrupt flag */

        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        if(CANmodule->CANtxCount > 0U){
            uint16_t i;             /* index of transmitting message */

            /* first buffer */
            CO_CANtx_t *buffer = &CANmodule->txArray[0];
            /* search through whole array of pointers to transmit message buffers. */
            for(i = CANmodule->txSize; i > 0U; i--){
                /* if message buffer is full, send it. */
                if(buffer->bufferFull){
                    if (can_transmit(CANmodule->CANport, buffer->ident, false, false, buffer->DLC, buffer->data) != -1) {
                        buffer->bufferFull = false;
                        CANmodule->CANtxCount--;
                        /* Copy message to CAN buffer */
                        CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    /* if all transmit mailboxes are full, leave message unsent until next interrupt */
                    } else {
                        /* canSend... */
                        break;                      /* exit for loop */
                    }
                }
                buffer++;
            }/* end of for loop */

            /* Clear counter if no more messages */
            if(i == 0U){
                CANmodule->CANtxCount = 0U;
                can_disable_irq(CANmodule->CANport, CAN_IER_TMEIE);
            }
        }
    }
    else{
        /* some other interrupt reason */
    }
}