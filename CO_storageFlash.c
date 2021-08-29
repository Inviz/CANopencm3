/*
 * CANopen data storage object for storing data into block device (flash)
 *
 * @file        CO_storageFlash.c
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
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

#include "CO_storageFlash.h"
// #include "CO_flash.h"
#include <libopencm3/stm32/flash.h>

#include "301/crc16-ccitt.h"

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE

/* Total flash available to device */
#define CO_STORAGE_FLASH_DEVICE_SIZE 65536  // 131072
/* Flash page size */
#define CO_STORAGE_FLASH_PAGE_SIZE 1024
/* Allocated storage memory */
#define CO_STORAGE_FLASH_SIZE 16384

// use memory from the end of the device
#define CO_STORAGE_FLASH_BASE_ADDRESS \
  (void *)FLASH_BASE + CO_STORAGE_FLASH_DEVICE_SIZE - CO_STORAGE_FLASH_SIZE
#define CO_STORAGE_FLASH_OOB_ADDRESS \
  (void *)CO_STORAGE_FLASH_BASE_ADDRESS + CO_STORAGE_FLASH_DEVICE_SIZE
#define CO_STORAGE_FLASH_HEADER_SIZE sizeof(CO_storage_flash_record_t)
#define CO_STORAGE_FLASH_UNDEFINED_CRC 0xFFFFFFFF
/*
 * Function for writing data on "Store parameters" command - OD object 1010
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
void *current_record_address;

typedef struct {
  uint32_t crc;
  uint16_t index;
  uint16_t len;
} CO_storage_flash_record_t;

ODR_t storeFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule) {
  (void)CANmodule; /* unused */
  return CO_flash_write_entry(entry) == CO_ERROR_NO ? ODR_OK : ODR_HW;
}

/*
 * Function for restoring data on "Restore default parameters" command - OD 1011
 *
 * For more information see file CO_storage.h, CO_storage_entry_t.
 */
ODR_t restoreFlash(CO_storage_entry_t *entry, CO_CANmodule_t *CANmodule) {
  (void)CANmodule; /* unused */
  return CO_flash_erase_entry(entry) == CO_ERROR_NO ? ODR_OK : ODR_HW;
}

/******************************************************************************/
CO_ReturnError_t CO_storageFlash_init(CO_storage_t *storage,
                                      CO_CANmodule_t *CANmodule,
                                      OD_entry_t *OD_1010_StoreParameters,
                                      OD_entry_t *OD_1011_RestoreDefaultParam,
                                      CO_storage_entry_t *entries,
                                      uint8_t entriesCount,
                                      uint32_t *storageInitError) {
  CO_ReturnError_t ret;

  /* verify arguments */
  if (storage == NULL || entries == NULL || entriesCount == 0 ||
      storageInitError == NULL) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  storage->enabled = false;

  /* initialize storage and OD extensions */
  ret = CO_storage_init(storage, CANmodule, OD_1010_StoreParameters,
                        OD_1011_RestoreDefaultParam, storeFlash, restoreFlash,
                        entries, entriesCount);
  if (ret != CO_ERROR_NO) {
    return ret;
  }

  // assign runtime values to storage entries
  for (uint8_t i = 0; i < entriesCount; i++) {
    CO_storage_entry_t *entry = &entries[i];
    entry->index = i;
    entry->storage = storage;
  }

  if (CO_flash_read_all_entries(entries, entriesCount) != CO_ERROR_NO) {
    *storageInitError = 1;
  } else {
    *storageInitError = 0;
  }

  storage->enabled = true;
  return ret;
}

// scan all records in the storage, and use their last ever found value
CO_ReturnError_t CO_flash_read_all_entries(CO_storage_entry_t *entries,
                                           uint8_t entriesCount) {
  for (current_record_address = CO_STORAGE_FLASH_BASE_ADDRESS;
       current_record_address < CO_STORAGE_FLASH_OOB_ADDRESS;) {
    CO_storage_flash_record_t *record =
        (CO_storage_flash_record_t *)current_record_address;

    // reached last entry in the storage
    if (record->crc == CO_STORAGE_FLASH_UNDEFINED_CRC) {
      break;
    }

    // update entry with the last found value
    CO_storage_entry_t *entry = &entries[record->index];
    entry->addrFlash = record->len == 0 ? NULL
                                        : current_record_address +
                                              CO_STORAGE_FLASH_HEADER_SIZE;
    current_record_address += CO_STORAGE_FLASH_HEADER_SIZE + record->len;
  }

  // update length for all non-erased entries
  for (uint16_t i = 0; i < entriesCount; i++) {
    CO_storage_entry_t *entry = &entries[i];
    if (entry->addrFlash != NULL) {
      CO_storage_flash_record_t *record =
          (CO_storage_flash_record_t *)(current_record_address -
                                        CO_STORAGE_FLASH_HEADER_SIZE);
      entry->len = record->len;
    }
  }

  return CO_ERROR_NO;
}

// remove all duplicate records from storage
static CO_ReturnError_t CO_flash_compact_storage(CO_storage_t *storage) {
  uint16_t first_invalid_entry = 0;
  void *first_dirty_page_address;
  void *last_valid_address = CO_STORAGE_FLASH_BASE_ADDRESS;
  void *current_writing_address = CO_STORAGE_FLASH_BASE_ADDRESS;

  // determine how many pages dont need to be changed
  for (uint16_t i = 0; i < storage->entriesCount; i++) {
    CO_storage_entry_t *entry = &storage->entries[i];
    CO_storage_flash_record_t storage_record = {
        entry->index, CO_flash_generate_crc(entry), entry->len};

    // Check if meta record is valid
    // Worst edge case: First invalid meta record is split between two pages,
    // while only part on second page is is dirty: both pages will be
    // invalidated
    if (memcmp(last_valid_address, &storage_record, sizeof(storage_record)) !=
        0) {
      first_invalid_entry = i;
      break;
    }
    last_valid_address += CO_STORAGE_FLASH_HEADER_SIZE;

    for (uint16_t entry_bytes_correct = 0; entry_bytes_correct < entry->len;) {
      uint16_t page_bytes_left =
          CO_STORAGE_FLASH_PAGE_SIZE -
          ((uint32_t)last_valid_address % CO_STORAGE_FLASH_PAGE_SIZE);
      uint16_t page_bytes_used_by_entry =
          entry->len > page_bytes_left ? page_bytes_left : entry->len;

      // new data is too big to fit the storage, have to abort
      if (last_valid_address + page_bytes_used_by_entry >=
          CO_STORAGE_FLASH_OOB_ADDRESS) {
        return CO_ERROR_OUT_OF_MEMORY;
      }

      // skip all valid segments in case value intersects page boundary
      if (memcmp(last_valid_address, entry->addr + entry_bytes_correct,
                 page_bytes_used_by_entry) == 0) {
        entry_bytes_correct += page_bytes_used_by_entry;
        last_valid_address += page_bytes_used_by_entry;

        // invalid segment is found, loop can be stopped
      } else {
        first_invalid_entry = i;
        i = storage->entriesCount;
        break;
      }
    }

    // all entries are stored and memory is already compacted, changes are
    // unnecessary
    if (i == storage->entriesCount - 1) {
      return CO_ERROR_NO;
    }
  }

  // clear up all pages that are not exactly equal to compacted memory
  first_dirty_page_address =
      last_valid_address -
      ((uint32_t)last_valid_address % CO_STORAGE_FLASH_PAGE_SIZE);
  for (void *dirty_page_address = first_dirty_page_address;
       dirty_page_address < CO_STORAGE_FLASH_OOB_ADDRESS;
       dirty_page_address += CO_STORAGE_FLASH_PAGE_SIZE) {
    flash_unlock();
    flash_erase_page((uint32_t)dirty_page_address);
    flash_lock();
  }

  // start writing updates
  current_writing_address = last_valid_address;
  for (uint16_t i = first_invalid_entry; i < storage->entriesCount; i++) {
    CO_storage_entry_t *entry = &storage->entries[i];

    // first dirty entry may have valid record that doesnt need to be written
    if (current_writing_address + CO_STORAGE_FLASH_HEADER_SIZE >
        last_valid_address) {
      CO_storage_flash_record_t storage_record = {CO_flash_generate_crc(entry),
                                                  entry->index, entry->len};
      memcpy(current_writing_address, &storage_record,
             CO_STORAGE_FLASH_HEADER_SIZE);
      current_writing_address += CO_STORAGE_FLASH_HEADER_SIZE;
    }

    for (uint16_t entry_bytes_correct = 0; entry_bytes_correct < entry->len;) {
      uint16_t page_bytes_left =
          CO_STORAGE_FLASH_PAGE_SIZE -
          ((uint32_t)current_writing_address % CO_STORAGE_FLASH_PAGE_SIZE);
      uint16_t page_bytes_used_by_entry =
          entry->len > page_bytes_left ? page_bytes_left : entry->len;

      // first dirty entry may have valid segments in case it crosses page
      // boundary
      if (current_writing_address + page_bytes_used_by_entry >
          last_valid_address) {
        memcpy((void *)current_writing_address,
               entry->addr + entry_bytes_correct, page_bytes_used_by_entry);
      }
      entry_bytes_correct += page_bytes_used_by_entry;
      current_writing_address += page_bytes_used_by_entry;
    }
  }

  // remap entry addresses to newly compacted data
  return CO_flash_read_all_entries(storage->entries, storage->entriesCount);
}

CO_ReturnError_t CO_flash_write_entry(CO_storage_entry_t *entry) {
  // if there's no more room to append entry to, memory has to be compacted and
  // that will write the entry
  if (current_record_address + entry->len + CO_STORAGE_FLASH_HEADER_SIZE >=
      CO_STORAGE_FLASH_OOB_ADDRESS) {
    return CO_flash_compact_storage(entry->storage);
  }

  // bail out if entry doesnt need to be updated
  CO_storage_flash_record_t storage_record = {CO_flash_generate_crc(entry),
                                              entry->index, entry->len};
  if (memcmp(entry->addrFlash, &storage_record, CO_STORAGE_FLASH_HEADER_SIZE) ==
          0 &&
      memcmp(entry->addrFlash + sizeof(storage_record), entry->addr,
             entry->len) == 0) {
    return CO_ERROR_NO;
  }

  // append new entry in the allocated space
  entry->addrFlash = current_record_address;
  current_record_address += CO_STORAGE_FLASH_HEADER_SIZE + entry->len;
  CO_ReturnError_t status =
      CO_flash_write(entry->addrFlash, (uint8_t *)&storage_record,
                     CO_STORAGE_FLASH_HEADER_SIZE);
  if (status != CO_ERROR_NO) return status;
  status = CO_flash_write(entry->addrFlash + CO_STORAGE_FLASH_HEADER_SIZE,
                          entry->addr, entry->len);
  if (status != CO_ERROR_NO) return status;

  return CO_ERROR_NO;
}

uint8_t CO_flash_erase_entry(CO_storage_entry_t *entry) {
  CO_storage_flash_record_t storage_record = {CO_flash_generate_crc(entry),
                                              entry->index, 0};
  if (memcmp(entry->addrFlash, (void *)&storage_record,
             CO_STORAGE_FLASH_HEADER_SIZE) == 0) {
    return CO_ERROR_NO;
  }
  entry->addrFlash = current_record_address;
  current_record_address += CO_STORAGE_FLASH_HEADER_SIZE + entry->len;
  return CO_flash_write(entry->addrFlash, (uint8_t *)&storage_record,
                        CO_STORAGE_FLASH_HEADER_SIZE);
}

CO_ReturnError_t CO_flash_generate_crc(CO_storage_entry_t *entry) {
  (void)entry; /* unused */
  return 1;
}

CO_ReturnError_t CO_flash_write(uint8_t *address, uint8_t *data, uint16_t len) {
  uint32_t starting_address = (uint32_t)address;
  flash_unlock();
  for (int i = 0; i < len; i += 2) {
    flash_program_half_word(starting_address + i, *((uint16_t *)(data + i)));
    uint32_t flash_status = flash_get_status_flags();
    if (flash_status != FLASH_SR_EOP) {
      flash_clear_status_flags();
      return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    if (*((uint16_t *)(starting_address + i)) != *((uint16_t *)(data + i))) {
      return CO_ERROR_DATA_CORRUPT;
    }
  }
  flash_lock();
  return CO_ERROR_NO;
}

/******************************************************************************/
// Function has to be called periodically in order for writes to persist
void CO_storageFlash_auto_process(CO_storage_t *storage) {
  /* verify arguments */
  if (storage == NULL || !storage->enabled) {
    return;
  }

  /* loop through entries */
  for (uint8_t i = 0; i < storage->entriesCount; i++) {
    CO_storage_entry_t *entry = &storage->entries[i];

    if ((entry->attr & CO_storage_auto) == 0) continue;

    // write data lazily
    CO_flash_write_entry(entry);
  }
}

#endif /* (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE */
