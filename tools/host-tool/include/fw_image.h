#ifndef FW_IMAGE_H
#define FW_IMAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* io-edge-hub-rust (embassy-boot) 升级载荷:
 *   [裸 app 二进制][ed25519 签名 64B]   —— tools/sign.py 产出的 app.dfu.bin
 * keyhash 不在镜像内 —— ed25519 验签在设备侧完成, START 时校验的 keyhash 是
 * 编译期常量 SHA-256(公钥), 必须与固件 proto::fw_upg::FW_KEYHASH 一致
 * (换钥匙: tools/gen_ed25519.py 重新生成后同步常量并重编两端)。 */
#define IMG_SIG_LEN     64
#define IMG_MIN_SIZE    (IMG_SIG_LEN + 1)
#define IMG_MAX_SIZE    0x80000u   /* 设备 DFU 暂存分区 512 KiB */
#define IMG_KEYHASH_LEN 32

/* 旧 MCUboot 镜像 magic (识别后明确拒绝, 提示换载荷) */
#define IMG_MCUBOOT_MAGIC 0x96f3b83d

/*
 * 校验载荷合法性: 长度能容纳尾部 64B 签名且不超过 DFU 分区。
 * 用于在升级前拒绝非固件文件 (任意二进制/文本等)。
 * @return true=合法升级载荷; false=过短/超过 DFU 分区
 */
bool fw_image_validate_header(const uint8_t *data, size_t len);

/* 识别旧 MCUboot 签名镜像 (magic 0x96F3B83D): 与本工具协议不匹配,
 * 用于给出精确的错误提示而不是笼统的 "非固件镜像"。 */
bool fw_image_is_mcuboot(const uint8_t *data, size_t len);

/* 32B keyhash 常量 (SHA-256 of ed25519 public key), 直接发给设备 START。
 * 生命周期内不变, 与固件同步更新。 */
const uint8_t *fw_image_keyhash(void);

#endif /* FW_IMAGE_H */
