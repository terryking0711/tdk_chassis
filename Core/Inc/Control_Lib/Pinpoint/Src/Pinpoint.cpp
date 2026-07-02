#include <Control_Lib/Pinpoint/Inc/Pinpoint.hpp>

extern I2C_HandleTypeDef hi2c1;
volatile bool i2c_req = 1;
bool i2c_reconnect = 0;
bool i2c_connected = 1;
PinpointI2C pinpoint(&hi2c1);
PinpointI2C::BulkData bd;
bool initi_yet = 0;

// ---- 建構 ----
PinpointI2C::PinpointI2C(I2C_HandleTypeDef* hi2c, uint8_t addr7bit, uint32_t timeoutMs)
: hi2c_(hi2c),
  devAddr_(static_cast<uint16_t>(addr7bit) << 1), // HAL 多數範例使用 8-bit 位址（左移 1）
  timeoutMs_(timeoutMs)
{}

void PinpointI2C::Pinpoint_Init()
{
	initi_yet = 1;
    // 1) 連線檢測
    if (!ping()) {
        // TODO: 報警或重試
    }
    // 2) 一次性配置（每次上電後都要重新送配置；手冊明確說明）
    setTicksPerMM(19.894f);       // 依你的 pod 實際數值
    setOffsets(75.0f, -65.0f);  // 依你的機構量測
    // pinpoint.setYawScalar(1.0f);        // 通常不建議改動
    // 3) 方向確認（前+X、左+Y）
    setEncoderDirections(PinpointI2C::EncDir::Forward,
                                  PinpointI2C::EncDir::Reversed);
    // 4) 開賽或程式啟動時：確保靜止後歸零+IMU校正（~0.25s）
    resetPosAndIMU();

}
void PinpointI2C::Pinpoint_TaskLoop()
{
    // a) 用 Bulk 取資料（最省 I²C 次數）
		if (readBulk(bd)) {
			i2c_connected = 1;

			// 可依 bd.device_status 判斷 Ready/故障
			// Ready: bit0=1；Calibrating: bit1；X/Y pod not detected: bit2/bit3
		} else {
			// 連續錯誤累計，可用 pinpoint.isConnected() 做健康狀態
			if (!isConnected()) {
	        	i2c_connected = 0;
				HAL_I2C_DeInit(&hi2c1);
				HAL_I2C_Init (&hi2c1);
				i2c_reconnect = ping();
				// TODO: 重置 I2C、重新 ping、告警等
			}else{
	        	//i2c_connected = 1;

			}
		}
}
// ---- 連線檢測 ----
void PinpointI2C::setMaxConsecutiveErrors(uint8_t n) { maxConsecutiveErrors_ = n ? n : 1; }
uint8_t PinpointI2C::consecutiveErrors() const { return consecutiveErrors_; }

bool PinpointI2C::isConnected() const {
    return consecutiveErrors_ < maxConsecutiveErrors_;
}

bool PinpointI2C::ping() {
    uint32_t id = 0;
    if (!readDeviceID(id)) return false;
    return (id == 2u); // 手冊：Device ID 應為 2
}

// ---- 公用讀寫 ----
bool PinpointI2C::regRead(uint8_t reg, void* buf, uint16_t len) {
    if (HAL_I2C_Mem_Read(hi2c_, devAddr_, reg, I2C_MEMADD_SIZE_8BIT,
                         static_cast<uint8_t*>(buf), len, timeoutMs_) == HAL_OK) {
        onOk();
        return true;
    }
    onErr();
    return false;
}

bool PinpointI2C::regWrite(uint8_t reg, const void* buf, uint16_t len) {
    if (HAL_I2C_Mem_Write(hi2c_, devAddr_, reg, I2C_MEMADD_SIZE_8BIT,
                          const_cast<uint8_t*>(static_cast<const uint8_t*>(buf)), len, timeoutMs_) == HAL_OK) {
        onOk();
        return true;
    }
    onErr();
    return false;
}

bool PinpointI2C::writeU32(uint8_t reg, uint32_t v) { return regWrite(reg, &v, sizeof(v)); }
bool PinpointI2C::readU32(uint8_t reg, uint32_t& v) { return regRead(reg, &v, sizeof(v)); }
bool PinpointI2C::writeF32(uint8_t reg, float v)    { return regWrite(reg, &v, sizeof(v)); }
bool PinpointI2C::readF32(uint8_t reg, float& v)    { return regRead(reg, &v, sizeof(v)); }

void PinpointI2C::onOk()  { if (consecutiveErrors_ > 0) consecutiveErrors_ = 0; }
void PinpointI2C::onErr() { if (consecutiveErrors_ < 0xFF) ++consecutiveErrors_; }

// ---- 基礎暫存器 ----
bool PinpointI2C::readDeviceID(uint32_t& id)      { return readU32(REG_DEVICE_ID, id); }
bool PinpointI2C::readDeviceVersion(uint32_t& v)  { return readU32(REG_DEVICE_VER, v); }
bool PinpointI2C::readDeviceStatus(uint32_t& st)  { return readU32(REG_DEVICE_STATUS, st); }
bool PinpointI2C::readLoopTime(uint32_t& us)      { return readU32(REG_LOOP_TIME, us); }

bool PinpointI2C::readRawEncoders(uint32_t& encX, uint32_t& encY) {
    bool okX = readU32(REG_ENC_X, encX);
    bool okY = readU32(REG_ENC_Y, encY);
    return okX && okY;
}

// ---- Bulk Read ----
bool PinpointI2C::readBulk(BulkData& out) {
    // 手冊：Bulk Read 總長 40 bytes，內容順序對應文件（此處採用常見版本的排列）
    uint8_t buf[40] = {0};
    if (!regRead(REG_BULK, buf, sizeof(buf))) return false;

    // 解析：依序 4-byte 對齊（uint32/float）
    std::memcpy(&out.device_status, &buf[0], 4);
    std::memcpy(&out.loop_time_us,  &buf[4], 4);
    std::memcpy(&out.enc_x_raw,     &buf[8], 4);
    std::memcpy(&out.enc_y_raw,     &buf[12],4);
    std::memcpy(&out.pos_x_mm,      &buf[16],4);
    std::memcpy(&out.pos_y_mm,      &buf[20],4);
    std::memcpy(&out.heading_rad,   &buf[24],4);
    std::memcpy(&out.vel_x_mm_s,    &buf[28],4);
    std::memcpy(&out.vel_y_mm_s,    &buf[32],4);
    std::memcpy(&out.vel_h_rad_s,   &buf[36],4);
    return true;
}

// ---- 位置/速度 ----
bool PinpointI2C::readPosition(Pose& p) {
    return readF32(REG_POS_X, p.x_mm) &&
           readF32(REG_POS_Y, p.y_mm) &&
           readF32(REG_POS_H, p.heading);
}

bool PinpointI2C::readVelocity(Velocity& v) {
    return readF32(REG_VEL_X, v.vx_mm_s) &&
           readF32(REG_VEL_Y, v.vy_mm_s) &&
           readF32(REG_VEL_H, v.w_rad_s);
}

// ---- 設定 ----
bool PinpointI2C::setTicksPerMM(float tpm) {
    return writeF32(REG_TICKS_PER_MM, tpm);
}

bool PinpointI2C::setOffsets(float x_offset_mm, float y_offset_mm) {
    bool ok1 = writeF32(REG_X_OFFSET, x_offset_mm);
    bool ok2 = writeF32(REG_Y_OFFSET, y_offset_mm);
    return ok1 && ok2;
}

bool PinpointI2C::setYawScalar(float yaw_scalar) {
    return writeF32(REG_YAW_SCALAR, yaw_scalar);
}

// ---- 控制 ----
bool PinpointI2C::resetIMU() {
    uint32_t cmd = CTRL_RESET_IMU;
    return writeU32(REG_DEVICE_CTRL, cmd);
}

bool PinpointI2C::resetPosAndIMU() {
    uint32_t cmd = CTRL_RESET_POS_AND_IMU;
    return writeU32(REG_DEVICE_CTRL, cmd);
}

bool PinpointI2C::setEncoderDirections(EncDir x, EncDir y) {
    uint32_t cmd = 0;
    cmd |= (y == EncDir::Reversed) ? CTRL_SET_Y_REV : CTRL_SET_Y_FWD;
    cmd |= (x == EncDir::Reversed) ? CTRL_SET_X_REV : CTRL_SET_X_FWD;
    return writeU32(REG_DEVICE_CTRL, cmd);
}

// ---- setPosition（覆寫位置暫存器）----
bool PinpointI2C::writePosition(const Pose& p) {
    bool okx = writeF32(REG_POS_X, p.x_mm);
    bool oky = writeF32(REG_POS_Y, p.y_mm);
    bool okh = writeF32(REG_POS_H, p.heading);
    return okx && oky && okh;
}
