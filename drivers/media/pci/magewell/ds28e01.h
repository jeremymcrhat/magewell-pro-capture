/*
* Standard GPL header here!!! 
***/

void ds28e01_reset(struct ds28e01_device *dev);
void ds28e01_skip_rom(struct ds28e01_device *dev);
void ds28e01_read_block(struct ds28e01_device *dev, u8 *data, u32 count);
void ds28e01_write_block(struct ds28e01_device *dev, const u8 *data, u32 count);
void ds28e01_read_memory(struct ds28e01_device *dev, u8 *data, u16 addr, u32 count);
bool ds28e01_authenticate(struct ds28e01_device *dev);
