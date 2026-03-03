
typedef enum KEYS
{
    NOPRESS = -1,
    LEFT = 0x80,
    RIGHT,
    UP,
    DOWN,
    EXIT
};

KEYS getKey(bool bBlocking = true);
void dumpBinary(uint8_t *input, uint16_t len, uint8_t width = 8);


