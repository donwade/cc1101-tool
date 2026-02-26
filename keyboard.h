
typedef enum KEYS
{
    NOPRESS = -1,
    LEFT = 0x80,
    RIGHT,
    UP,
    DOWN
};

KEYS getKey(bool bBlocking);


