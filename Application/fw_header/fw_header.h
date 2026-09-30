/**
 * @file  fw_header.h
 * @brief Firmware image header embedded at APP_FLASH_BASE + 0x200.
 *
 * The header is placed in the dedicated linker section ".fw_header" which is
 * located at exactly 0x200 bytes from the start of FLASH (right after the
 * vector table padding).  The bootloader reads this header to verify product
 * identity before accepting or running the image.
 *
 * All fields are baked in at compile time via CMake -D flags.
 * CRC32 and image size are NOT in the header — they travel in OTA metadata.
 *
 * hw_revision encoding: (major << 8) | minor
 *   major — increments on MCU pinout changes (requires new FW major)
 *   minor — increments on non-pinout HW changes (FW-compatible)
 *   patch  — cosmetic PCB changes, not stored in firmware
 *
 * fw_version encoding: (major << 8) | minor
 *   major — must equal hw_revision major for OTA acceptance
 *   minor — increments on firmware-only changes
 *
 * Layout must match fw_header_t in the bootloader's app_validate.h exactly.
 */
#ifndef FW_HEADER_H
#define FW_HEADER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Magic identifying a valid PLCJ firmware image ("PLCJ" in ASCII). */
#define FW_IMAGE_MAGIC          0x504C434Au

/* Byte offset from APP_FLASH_BASE where the header is located.
 * Must match FW_HEADER_OFFSET in the bootloader's app_validate.h. */
#define FW_HEADER_OFFSET        0x200u

/**
 * Firmware image header (packed, 28 bytes total).
 * Placed at APP_FLASH_BASE + FW_HEADER_OFFSET in the binary.
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;               /* FW_IMAGE_MAGIC ("PLCJ")              */
    uint32_t product_id;          /* Module type identifier (exact match) */
    uint16_t hw_revision;         /* (major << 8) | minor                 */
    uint16_t reserved0;
    uint32_t fw_version;          /* (major << 8) | minor                 */
    uint32_t vector_table_offset; /* Offset of vector table (always 0)    */
    uint32_t reserved1[2];
} fw_header_t;

/* Canonical module identity — the single source of truth for this module.
 * May still be overridden at build time via -DFW_PRODUCT_ID /
 * -DFW_HW_REVISION / -DFW_VERSION_VALUE (guarded below). Bump fw_version here. */
#ifndef FW_PRODUCT_ID
#define FW_PRODUCT_ID    0x504C0804u  /* PL, 8ch, AI current */
#endif
#ifndef FW_HW_REVISION
#define FW_HW_REVISION   0x0101u  /* hw:01.01 */
#endif
#ifndef FW_VERSION_VALUE
#define FW_VERSION_VALUE 0x0105u  /* fw:01.05 */
#endif

/** The single header instance placed in the .fw_header linker section. */
extern const fw_header_t g_fw_header;

#ifdef __cplusplus
}
#endif

#endif /* FW_HEADER_H */
