char *get_mode(uint8_t command, uint8_t *tempData);
void set_mode(uint8_t newMode);
void sendMode(uint8_t src, uint8_t dst, uint8_t connection);
void CreatePacket(uint8_t src, uint8_t dst, uint8_t isTCP, uint8_t dataLength, uint8_t *data);
void sendRSSI(uint8_t src, uint8_t dst, uint8_t connection);
void getRSSI();
void getArray();
void reinitialize();
void initializeAdhocMode();
void moveForward();
char *get_data(uint8_t src, uint8_t dst, uint8_t *tempData);
void sendPacket(uint8_t src, uint8_t dst, uint8_t internal, uint8_t isTCP, uint8_t isACK, uint8_t counterHigh, uint8_t counterLow, uint8_t dataLength, uint8_t *data);
void sendToSlave(uint8_t src, uint8_t dst, uint8_t command);
void initialize();