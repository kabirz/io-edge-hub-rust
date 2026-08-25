#include "fw_image.h"
#include <string.h>

/* SHA-256(keys/ed25519.pub) — 与固件 crates/proto/src/fw_upg.rs 的
 * FW_KEYHASH、tools/fwupd_udp.py 的 KEYHASH 同源。换钥匙时三处同步。 */
static const uint8_t g_keyhash[IMG_KEYHASH_LEN] = {
    0x7c, 0xb9, 0xc1, 0xc5, 0x52, 0x4d, 0xf6, 0xbd,
    0xa9, 0x73, 0xb7, 0x51, 0xda, 0xd4, 0x20, 0x1d,
    0x2d, 0x5d, 0x89, 0x08, 0x66, 0x8e, 0xaf, 0xdf,
    0x44, 0x19, 0x23, 0x99, 0x69, 0xc6, 0x85, 0x1f,
};

bool fw_image_validate_header(const uint8_t *data, size_t len)
{
    /* 载荷 = 裸镜像 + 尾部 64B ed25519 签名: 只需长度合法性
     * (> 签名本身, <= 设备 DFU 分区)。签名内容由设备端验签。 */
    return data != NULL && len >= IMG_MIN_SIZE && len <= IMG_MAX_SIZE;
}

bool fw_image_is_mcuboot(const uint8_t *data, size_t len)
{
    if (!data || len < 4) {
        return false;
    }
    uint32_t magic = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    return magic == IMG_MCUBOOT_MAGIC;
}

const uint8_t *fw_image_keyhash(void)
{
    return g_keyhash;
}
